from flask import Flask, send_from_directory, request, jsonify, send_file
import tempfile
import os
import json
from flask_cors import CORS
from werkzeug.utils import safe_join
import time
import uuid

# Import the modified kerf function
from kerf_backend import generate_kerf_dxf

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Directory to store temporary files
TEMP_DIR = tempfile.gettempdir()


@app.route('/')
def home():
    return send_from_directory('static', 'index.html')  # Serves your HTML

@app.route('/api/generate-kerf', methods=['POST'])
def generate_kerf():
    try:
        data = request.json
        print("called")
        print(f"'generate-kerf' request received, values: {data['toolType']}, {data['coneAngle']}")
        # Extract parameters from request
        control_points = data['controlPoints']
        curve_type = data['curveType']  # "bezier" or "spline"
        tool_type = data['toolType']   # saw or cone
        cut_width = float(data['cutWidth'])
        cone_angle = float(data['coneAngle'])
        cut_depth = float(data['cutDepth'])
        line_length = float(data['lineLength'])
        offset = float(data['offset'])
        
        # Optional parameters with defaults
        search_window = int(data.get('searchWindow', 40))
        curve_samples = int(data.get('curveSamples', 600))
        spline_tension = float(data.get('splineTension', 0.5))
        
        # Generate a unique filename
        unique_id = str(uuid.uuid4())
        temp_dxf_path = os.path.join(TEMP_DIR, f"kerf_{unique_id}.dxf")
        
        # Call the kerf function
        cut_distances, total_length, num_cuts = generate_kerf_dxf(
            control_points=control_points,
            curve_type=curve_type,
            tool_type=tool_type,
            cone_angle=cone_angle,
            cut_width=cut_width,
            cut_depth=cut_depth,
            line_length=line_length,
            offset=offset,
            output_file=temp_dxf_path,
            display_extra_geometries=False,
            search_window=search_window,
            curve_samples=curve_samples,
            spline_tension=spline_tension
        )
        
        # Return both the file and the cut information
        result = {
            'cutDistances': cut_distances,
            'totalLength': total_length,
            'numCuts': num_cuts,
            #'kerfAngle': 2 * atan2(cut_width, 2 * cut_depth) * 180/pi  # in degrees
        }
        
        # Return the filename for download endpoint
        return jsonify({
            'result': result,
            'dxfPath': os.path.basename(temp_dxf_path)
        })
    
    except Exception as e:
        print(f"error: {e}")
        return jsonify({
            'error': str(e)
        }), 500

@app.route('/api/download-dxf/<filename>', methods=['GET'])
def download_dxf(filename):
    try:
        # Ensure the filename is safe
        filepath = safe_join(TEMP_DIR, filename)
        
        # Check if file exists
        if not os.path.exists(filepath):
            return jsonify({'error': 'File not found'}), 404
        
        # Serve the file
        return send_file(
            filepath,
            as_attachment=True,
            #attachment_filename=filename,
            mimetype='application/dxf'
        )
    
    except Exception as e:
        return jsonify({
            'API error': str(e)
        }), 500

# Cleanup old files periodically (optional)
@app.route('/api/cleanup', methods=['POST'])
def cleanup_old_files():
    try:
        # Delete files older than 1 hour
        current_time = time.time()
        count = 0
        
        for filename in os.listdir(TEMP_DIR):
            if filename.startswith('kerf_') and filename.endswith('.dxf'):
                filepath = os.path.join(TEMP_DIR, filename)
                file_age = current_time - os.path.getmtime(filepath)
                
                # If file is older than 1 hour (3600 seconds)
                if file_age > 3600:
                    os.remove(filepath)
                    count += 1
        
        return jsonify({'message': f'Removed {count} old files'})
    
    except Exception as e:
        return jsonify({
            'error': str(e)
        }), 500

if __name__ == '__main__':
    app.run(debug=True)