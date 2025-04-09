# Modified kerf.py to work as a function for the web API
import ezdxf
import numpy as np
import os
from ezdxf.math import Vec3, Vec2
from math import atan2, sqrt, pi, sin, cos, radians
from scipy import interpolate

def angle_diff(angle1, angle2):
    """Calculate the smallest difference between two angles in radians"""
    diff = (angle1 - angle2) % (2*pi)
    if diff > pi:
        diff = diff - 2*pi
    return diff

def create_curve_and_tangents(points, samples, spline_tension=0.5):
    """Create curve points and calculate tangent angles based on curve type"""
    if len(points) == 3:
        # Quadratic bezier curve handling
        return create_bezier_and_tangents(points, samples)
    elif len(points) > 3:
        # Spline curve handling
        return create_spline_and_tangents(points, samples, spline_tension)

def create_bezier_and_tangents(points, samples):
    """Create bezier curve points and calculate tangent angles"""
    p0, p1, p2 = map(Vec3, [(p[0], p[1], 0) for p in points])
    curve_points = []
    angles = []
    curvatures = []
    
    for t in np.linspace(0, 1, samples + 1):
        # Calculate point on curve
        point = (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
        curve_points.append(point)
        
        # Calculate tangent (first derivative)
        der = Vec3(2 * ((1 - t) * (p1 - p0) + t * (p2 - p1)))
        angle_rad = atan2(der.y, der.x) % (2*pi)
        angles.append(angle_rad)
        
        # Calculate curvature sign (using second derivative)
        der2 = Vec3(2 * (p2 - 2*p1 + p0))
        # Cross product of first and second derivatives gives curvature direction
        cross_z = der.x * der2.y - der.y * der2.x
        curvatures.append(np.sign(cross_z))
        
    return curve_points, angles, curvatures

def create_spline_and_tangents(points, samples, spline_tension=0.5):
    """Create cubic spline curve and calculate tangent angles"""
    # Extract x and y coordinates
    x_points = np.array([p[0] for p in points])
    y_points = np.array([p[1] for p in points])
    
    # Create a parameter array based on cumulative chord length
    t = np.zeros(len(points))
    for i in range(1, len(points)):
        t[i] = t[i-1] + sqrt((x_points[i] - x_points[i-1])**2 + (y_points[i] - y_points[i-1])**2)
    
    # Normalize parameter values
    if t[-1] > 0:
        t = t / t[-1]
    
    # Create cubic spline interpolations with tension control
    # The lower the smoothing factor (s), the closer to interpolation
    # Higher values create smoother curves that may not pass through all points
    smoothing_factor = (1 - spline_tension) * len(points)
    
    # Create spline for x and y coordinates
    tck_x = interpolate.splrep(t, x_points, s=smoothing_factor)
    tck_y = interpolate.splrep(t, y_points, s=smoothing_factor)
    
    # Generate points along the spline
    u = np.linspace(0, 1, samples + 1)
    x_spline = interpolate.splev(u, tck_x)
    y_spline = interpolate.splev(u, tck_y)
    
    # Get derivatives for tangent angles and curvature
    x_der1 = interpolate.splev(u, tck_x, der=1)
    y_der1 = interpolate.splev(u, tck_y, der=1)
    x_der2 = interpolate.splev(u, tck_x, der=2)
    y_der2 = interpolate.splev(u, tck_y, der=2)
    
    # Create curve points
    curve_points = [Vec3(x, y, 0) for x, y in zip(x_spline, y_spline)]
    
    # Calculate tangent angles
    angles = [atan2(dy, dx) % (2*pi) for dx, dy in zip(x_der1, y_der1)]
    
    # Calculate curvature signs
    curvatures = [np.sign(dx * ddy - dy * ddx) 
                 for dx, dy, ddx, ddy in zip(x_der1, y_der1, x_der2, y_der2)]
    
    return curve_points, angles, curvatures

def draw_curve(modelspace, points, color=0):
    """Draw a curve connecting the given points"""
    for i in range(len(points)-1):
        modelspace.add_line(points[i], points[i+1], dxfattribs={"color": color})

def two_p_distance(p1, p2):
    """Calculate distance between two points"""
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def find_next_cut_index(current_index, target_angle, angles, curve_points, search_window, direction=1, min_distance=5.0):
    """Find the index of the next cut point efficiently by searching in a limited window"""
    start_idx = max(0, current_index - search_window if direction < 0 else current_index)
    end_idx = min(len(angles), current_index + search_window if direction > 0 else current_index)
    
    search_range = range(start_idx, end_idx) if direction > 0 else range(end_idx-1, start_idx-1, -1)
    
    best_idx = current_index
    min_diff = float('inf')
    
    for i in search_range:
        if i == current_index:
            continue
        
        # Skip if the point is too close to the current one (prevents tight clustering)
        dist = two_p_distance(curve_points[i], curve_points[current_index])
        if dist < min_distance:
            continue
            
        diff = abs(angle_diff(angles[i], target_angle))
        if diff < min_diff:
            min_diff = diff
            best_idx = i
    
    # If we couldn't find a good match in our window, try extending the search
    if best_idx == current_index and search_window < len(angles)//4:
        return find_next_cut_index(current_index, target_angle, angles, curve_points, 
                            search_window * 2, direction, min_distance)
    
    return best_idx

def generate_kerf_dxf(control_points, curve_type, tool_type, cone_angle, cut_width, cut_depth, line_length, offset, output_file, 
                      display_extra_geometries=False, search_window=80, curve_samples=600, spline_tension=0.5):
    """Generate a kerf pattern DXF file and return cut information"""
    # Calculate "kerf angle" in radians - the angle by which a single cut bends the material
    if tool_type == "saw": kerf_angle_rad = 2 * atan2(cut_width, 2 * cut_depth)
    else:  kerf_angle_rad = radians(cone_angle)   #I will need to check if this is ok geometrically
    
    # Initialize DXF document
    doc = ezdxf.new()
    msp = doc.modelspace()
    layers = doc.layers
    layers.add(name="CutsSide1", color=0)
    layers.add(name="CutsSide2", color=1)
    layers.add(name="Start and end of workpiece", color=6)
    
    # Process control points
    # Convert list of [x,y] to list of (x,y) if needed
    cpoints = []
    for p in control_points:
        if isinstance(p, list):
            cpoints.append((p[0], p[1]))
        else:
            cpoints.append(p)
    
    # Generate curve points, tangent angles, and curvature signs
    if curve_type == "bezier":
        # Use only first 3 points for bezier
        cpoints = cpoints[:3]
        curve_points, tangent_angles, curvatures = create_curve_and_tangents(cpoints, curve_samples)
    else:  # spline
        curve_points, tangent_angles, curvatures = create_curve_and_tangents(cpoints, curve_samples, spline_tension)
    
    if display_extra_geometries:
        # Draw control points
        for point in cpoints:
            # Add z=0 if needed
            point_3d = (point[0], point[1], 0) if len(point) == 2 else point
            msp.add_point(point_3d, dxfattribs={"color": 3})
        # Draw curve
        draw_curve(msp, curve_points, 5)
    
    # Find cuts starting from the leftmost point
    leftmost_idx = min(range(len(curve_points)), key=lambda i: curve_points[i][0])
    
    cuts = [leftmost_idx]
    cuts_curvatures = [curvatures[leftmost_idx]]
    current_idx = leftmost_idx
    
    # Process the curve from left to right
    while current_idx < len(curve_points) - 1:
        # Determine the correct angle adjustment based on curvature
        current_angle = tangent_angles[current_idx]
        current_curvature = curvatures[current_idx]
        
        # Determine angle change direction based on curvature sign
        # For concave-up sections (positive curvature), add kerf angle
        # For concave-down sections (negative curvature), subtract kerf angle
        angle_adjustment = kerf_angle_rad * current_curvature
        next_angle = (current_angle + angle_adjustment) % (2*pi)
        
        # Find the next cut point with the desired tangent angle
        next_idx = find_next_cut_index(current_idx, next_angle, tangent_angles, curve_points, search_window, 1, cut_width*2)
        
        # If we're not making progress, break
        if next_idx <= current_idx or next_idx >= len(curve_points):
            break
            
        cuts.append(next_idx)
        cuts_curvatures.append(curvatures[next_idx])
        current_idx = next_idx
    
    # Process the curve from leftmost point backwards
    current_idx = leftmost_idx
    while current_idx > 0:
        # Determine the correct angle adjustment based on curvature
        current_angle = tangent_angles[current_idx]
        current_curvature = curvatures[current_idx]
        
        # For moving backwards, we need to flip the direction of the angle change
        angle_adjustment = -kerf_angle_rad * current_curvature
        next_angle = (current_angle + angle_adjustment) % (2*pi)
        
        # Find the next cut point with the desired tangent angle (moving backwards)
        next_idx = find_next_cut_index(current_idx, next_angle, tangent_angles, curve_points, search_window, -1, cut_width*2)
        
        # If we're not making progress, break
        if next_idx >= current_idx or next_idx < 0:
            break
            
        cuts.insert(0, next_idx)
        cuts_curvatures.insert(0, curvatures[next_idx])
        current_idx = next_idx
    
    # Extract the actual cut points from indices
    cut_points = [curve_points[idx] for idx in cuts]
    
    # If showing extra geometries
    if display_extra_geometries:
        # Draw curve through all cut points
        draw_curve(msp, cut_points, 1)
        
        # Draw perpendicular lines at cut points to visualize kerf
        for idx in cuts:
            point = curve_points[idx]
            angle = tangent_angles[idx]
            # Perpendicular direction
            perp_angle = (angle + pi/2) % (2*pi)
            x2 = point[0] + 30 * cos(perp_angle)
            y2 = point[1] + 30 * sin(perp_angle)
            msp.add_line(point, (x2, y2, 0), dxfattribs={"color": 2})
    
    # Calculate distances between adjacent cuts
    cuts_distances = [two_p_distance(cut_points[i], cut_points[i+1]) 
                      for i in range(len(cut_points) - 1)]
    
    # Draw the final cut lines
    current_x = -(sum(cuts_distances)/2)
    
    # Add the starting edge
    msp.add_line((current_x+offset, -15), (current_x+offset, line_length+15), dxfattribs={"layer": "Start and end of workpiece"})
    
    # Add the cuts
    for i in range(len(cuts_distances)):
        current_x += cuts_distances[i]
        
        # Determine the layer based on curvature
        # Use the curvature of the cut point to determine which side the cut should be on
        if i+1 < len(cuts_curvatures):  # Make sure we don't go out of bounds
            curvature = cuts_curvatures[i+1]
            layer = "CutsSide2" if curvature < 0 else "CutsSide1"
            
            # Draw the cut line
            msp.add_line((current_x+offset, 0), (current_x+offset, line_length), dxfattribs={"layer": layer})
    
    # Add the final right edge
    msp.add_line((current_x+offset, -15), (current_x+offset, line_length+15), dxfattribs={"layer": "Start and end of workpiece"})
    
    # Save the file
    doc.saveas(output_file)
    
    total_length = sum(cuts_distances)
    num_cuts = len(cut_points)
    
    return cuts_distances, total_length, num_cuts