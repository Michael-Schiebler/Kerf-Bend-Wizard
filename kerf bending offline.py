#Kerf bending parabolas, Michael Schiebler 2-4-25

"""ABSTRACT:
Kerf bending is a technique in which cuts (kerfs) are made partway through a material, typically plywood, to create a flexible hinge point,
allowing flat panels to be bent into curved surfaces.

Typically equally spaced lines are used, which result in the curves being arcs.

The algorithm implemented here computes the points at which to cut in order to obtain a kerf bent piece that follows the curve chosen. Cuts should be made exactly on
the lines outputted (without tool correction)
"""
# USER INPUT:

# CURVE DEFINITION
#these points act as control points of a quadratic bezier curve, which happens to be a portion of parabola
Cpoints = [(-400,0,0),(0,-820,0),(400,0,0)]

# MATERIAL AND TOOL INFO
#all dimensions in mm 

cut_width = 2.7   #thickness of the blade

cut_depth = 35  #ideally about  10–15% less than the thickness of your plywood, make sure to test it IRL beforehand.
                #Better approximations are obtained with deeper cut depths (thicker plywood) and thinner blades, this decreases the kerf angle and increase the amount of cuts needed
                #to conform to a given curve.

# OUTPUT PREFERENCES
line_length = 80   #the lenght of each line representing a cut

base_name = "kerf_dxf"  #default name with which the dxf file gets saved

display_extra_geometries = True   #draw all the extra stuff that isn't the lines needed for CAM. Namely the curve being traced and construction lines.
                                  #This will also help show the theory behind the algorithm and debug if needed.

# SYSTEM VARIABLES
Csamples= 1000  #number of samples taken along the bezier curve, keep it even!!, proportional to computational complexity
 
start_dxf = True  #whether or not to automatically open the output dxf (uses your default software)

start_dir = True  #automatically open the directory in which the file is located


#⚠️⚠️⚠️ DANGER LINE:  AUTHORIZED PERSONNEL ONLY BEYOND THIS POINT ⚠️⚠️⚠
#____________________________________________________________________________________________________________________________________________________________________


import ezdxf
import numpy as np
import os
from ezdxf.math import Vec3, Vec2
from math import radians, atan2, degrees, sin, cos, sqrt
import time

st = time.time()


def generate_filename():
    #this generates a unique filename to help with file management
    count = 1
    if not os.path.exists(f"{base_name}.dxf"):
        return f"{base_name}.dxf"
    while os.path.exists(f"{base_name}({count}).dxf"):
        count += 1
    return f"{base_name}({count}).dxf"

def create_quadratic_bezier(points, segment_number):
   # bezier with 2 control points, generates an array of points that lie on the curve, segment number referst to precision of approximation
    p0, p1, p2 = map(Vec3, points)
    points = []
    for t in np.linspace(0, 1, segment_number+1):
        points.append((1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2)
    return points
    
def draw_curve(modelspace, points, color=0):
    #connects points from point list
    for i in range(len(points)-1):
        modelspace.add_line(points[i], points[i+1], dxfattribs={"color": color})
        
def quadratic_bezier_tangent_angles(Cpoints, segment_number):
    p0, p1, p2 = map(Vec3, Cpoints)
    angles = []
    for t in np.linspace(0, 1, segment_number+1):
        # Calculate derivative (tangent vector) and ensure it's Vec3
        der = Vec3(2 * ((1-t) * (p1 - p0) + t * (p2 - p1)))
        # Calculate angle in radians, then convert to degrees
        angle_rad = atan2(der.y, der.x)
        angle_deg = degrees(angle_rad)
        # Normalize to 0-360 range
        angle_deg = angle_deg % 360
        angles.append(angle_deg)
    return angles

def draw_line(msp,pointx, pointy,angle, length):
        direction = Vec2.from_angle(radians(angle)) * length
        start=Vec2(pointx, pointy, 0)
        msp.add_line(
            start,
            end = (start + direction),
           # dxfattribs={"layer": curve } 
        )

def compute_line(msp,pointx, pointy,angle, length):
        direction = Vec2.from_angle(radians(angle)) * length
        start=Vec2(pointx, pointy, 0)
        return (start, start+direction)


def line_intersection(points1, points2):
    """Returns intersection point if the line segments intersect, else None"""
    # Unpack points (handles both Vec3 and array/tuple cases)
    (x1, y1), (x2, y2) = [(p.x, p.y) if hasattr(p, 'x') else (p[0], p[1]) for p in points1]
    (x3, y3), (x4, y4) = [(p.x, p.y) if hasattr(p, 'x') else (p[0], p[1]) for p in points2]
    
    # Line intersection math
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom == 0:  # Lines are parallel
        return None
    
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
    
    # Only return intersection if it lies within both segments
    if 0 <= ua <= 1 and 0 <= ub <= 1:
        x = x1 + ua * (x2 - x1)
        y = y1 + ua * (y2 - y1)
        return (x, y)
    return None


def two_p_distance(p1, p2):
    return sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def main():
    
    #calculate "kerf angle", the angle by which a single cut bends the material
    
    kerf_angle = degrees(2* atan2(cut_width, 2*cut_depth))
    print(f"Kerf angle: {kerf_angle:.2f}°")
    
    #initialize dxf document
    
    doc = ezdxf.new()
    msp = doc.modelspace()
    
    #draw control points   
    for i in Cpoints:
        msp.add_point(i, dxfattribs={"color": 3})
    
    # Create and draw bezier curve
    bezier_points = create_quadratic_bezier(Cpoints, Csamples)
    draw_curve(msp, bezier_points, 5)
    
    #initialize loop, start in middle
    current_pointx = bezier_points[int(len(bezier_points)/2)][0]
    current_pointy = bezier_points[int(len(bezier_points)/2)][1]
    tan_angles=quadratic_bezier_tangent_angles(Cpoints, Csamples)
    current_angle = kerf_angle/2 + tan_angles[int(len(bezier_points)/2)]

    #draw midpoint
    msp.add_point((current_pointx, current_pointy), dxfattribs={"color": 1})
    cuts =  []
    cuts.append((current_pointx,current_pointy))
    
    LineLenght = two_p_distance(Cpoints[0], Cpoints[1])/2    #this is needed due to lazy programming. This is the lenght of the lines drawn . The second line should be a ray of
    #LineLenght = 999999                                     # infinite lenght but I didn't feel like implementing it. This lenght should be enough, set to 99999 if there's any issue.
                                                             #It will be displyed in the dxf
    found_intersection = True
    while found_intersection:
        #sin and cos values are added to the start point in order to offset the line by a small amount, this allows it to only intersect the parabola in a single point
        line_coords = compute_line(msp, current_pointx+0.01*cos(radians(current_angle)), current_pointy+0.01*sin(radians(current_angle)), current_angle, LineLenght)  #this is sketchy because if the lenght is not enough it all fails, just put 999 for now but it displays incorrectly
        msp.add_line(line_coords[0],line_coords[1])
        #intersections:
        intersection = []
        found_intersection=False
        for j in range(len(bezier_points)-1):
            intersection = line_intersection(line_coords, (bezier_points[j], bezier_points[j+1]) )
            if intersection:
                cuts.append(intersection)
                msp.add_point(intersection, dxfattribs={"color": 1})
                current_pointx = intersection[0]
                current_pointy = intersection[1]
                current_angle = current_angle + kerf_angle
                #print(f"tang: {tan_angles[j]}  current : {current_angle}")
                found_intersection=True
                break

                
            
    #this adds the projection of the end point to the cuts array, not sure which type of projection is better
    old_coords = line_coords
    line_coords = compute_line(msp, Cpoints[2][0], Cpoints[2][1], 90 + current_angle, LineLenght)
    msp.add_line(line_coords[0],line_coords[1])
    cuts.append(line_intersection(line_coords, old_coords))
    #msp.add_point(line_intersection(line_coords, old_coords), dxfattribs={"color": 1})
        
    
    

    current_pointx = bezier_points[int(len(bezier_points)/2)][0]
    current_pointy = bezier_points[int(len(bezier_points)/2)][1]
    
    current_angle = 360 - (kerf_angle/2 + tan_angles[int(len(bezier_points)/2)])
    
    found_intersection = True
    while found_intersection:
        line_coords = compute_line(msp, current_pointx-0.01*cos(radians(current_angle)), current_pointy-0.01*sin(radians(current_angle)),180+current_angle, LineLenght)  #this is sketchy because if the lenght is not enough it all fails, just put 9999 for now but it displays incorrectly
        msp.add_line(line_coords[0],line_coords[1])
        intersection = []
        found_intersection=False
        for j in range(len(bezier_points)-1):
            intersection = line_intersection(line_coords, (bezier_points[j], bezier_points[j+1]) )
            if intersection:
                cuts.append(intersection)
                msp.add_point(intersection, dxfattribs={"color": 1})
                current_pointx = intersection[0]
                current_pointy = intersection[1]
                current_angle = (current_angle - kerf_angle)
                found_intersection=True
                break


    #this adds the projection of the start point to the cuts array, not sure which type of projection is better
    old_coords = line_coords
    line_coords = compute_line(msp, Cpoints[0][0], Cpoints[0][1], 90 + current_angle, LineLenght)
    msp.add_line(line_coords[0],line_coords[1])
    cuts.append(line_intersection(line_coords, old_coords))
    msp.add_point(line_intersection(line_coords, old_coords), dxfattribs={"color": 1})
    
    
    
    #this draws the cuts "flattened" as vertical so they can be imported directly into CAM software
    cuts.sort(key=lambda p: p[0])  # Order cuts left to right

    cuts_distances = [two_p_distance(cuts[i], cuts[i+1]) for i in range(len(cuts) - 1)]
    currentx = -(sum(cuts_distances)/2)   #this makes it so the cuts are drawn centered in the dxf (middle cut on origin)
    
    if not(display_extra_geometries):     #this resets the dxf document to erase all previously drawn entities
        doc = ezdxf.new()
        msp = doc.modelspace()
    
    for i in cuts_distances:
        if cuts_distances.index(i)==len(cuts_distances)/2:
            msp.add_line((currentx,0), (currentx, line_length), dxfattribs={"color": 6})
        elif cuts_distances.index(i)==0:
            msp.add_line((currentx,0), (currentx, line_length), dxfattribs={"color": 6})
        else:
            msp.add_line((currentx,0), (currentx, line_length), dxfattribs={"color": 0})
        currentx += i
    msp.add_line((currentx, 0), (currentx, line_length), dxfattribs={"color": 6})
    

    filename = generate_filename()
    doc.saveas(filename)
    print(f"Total lenght of workpiece: {sum(cuts_distances):.2f} mm. Number of cuts: {len(cuts)-2}")
    print(f"Saved as {filename}")
    
    et = time.time()
    print(f"Execution time: {(et - st)*1000:.2f} milliseconds")
    if start_dxf: os.startfile(filename)
    
if __name__ == "__main__":
    main()
