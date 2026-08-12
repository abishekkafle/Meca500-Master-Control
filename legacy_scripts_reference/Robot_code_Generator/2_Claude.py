import trimesh
import numpy as np

def slice_stl_to_gcode(stl_file, output_file, layer_height=0.2, print_speed=60, travel_speed=120, nozzle_temp=200, bed_temp=60):
    """
    Slice an STL file and generate G-code
    
    Args:
        stl_file (str): Path to input STL file
        output_file (str): Path to output G-code file
        layer_height (float): Layer height in mm
        print_speed (int): Printing speed in mm/s
        travel_speed (int): Travel speed in mm/s
        nozzle_temp (int): Nozzle temperature in °C
        bed_temp (int): Bed temperature in °C
    """
    
    # Load the STL file
    mesh = trimesh.load(stl_file)
    
    # Get the bounding box of the model
    min_z, max_z = mesh.bounds[:, 2]
    
    # Generate slice heights
    slice_heights = np.arange(min_z + layer_height, max_z, layer_height)
    
    # Open the output file
    with open(output_file, 'w') as f:
        # Write G-code header
        f.write("; G-code generated from STL slicer\n")
        f.write("; Layer height: {:.2f}mm\n".format(layer_height))
        f.write("\n")
        
        # Start G-code
        f.write("G21 ; Set units to millimeters\n")
        f.write("G90 ; Absolute positioning\n")
        f.write("M82 ; Extruder absolute mode\n")
        f.write("\n")
        
        # Set temperatures
        f.write("M104 S{} ; Set nozzle temperature\n".format(nozzle_temp))
        f.write("M140 S{} ; Set bed temperature\n".format(bed_temp))
        f.write("M190 S{} ; Wait for bed temperature\n".format(bed_temp))
        f.write("M109 S{} ; Wait for nozzle temperature\n".format(nozzle_temp))
        f.write("\n")
        
        # Home all axes
        f.write("G28 ; Home all axes\n")
        f.write("\n")
        
        # Initial setup
        f.write("G1 Z5 F5000 ; Move nozzle up\n")
        f.write("G1 X0 Y0 F{} ; Move to start position\n".format(travel_speed * 60))
        f.write("\n")
        
        # Slice the mesh at each height and generate G-code
        for i, height in enumerate(slice_heights):
            f.write("; Layer {} at Z={:.2f}\n".format(i+1, height))
            
            # Get the cross-section at this height
            section = mesh.section(plane_origin=[0, 0, height], plane_normal=[0, 0, 1])
            
            if section is not None:
                # Convert the section to 2D path
                path = section.to_planar()[0]
                
                # Move to layer height
                f.write("G1 Z{:.2f} F{}\n".format(height, travel_speed * 60))
                
                # Process each path in the section
                for path_idx, path_coords in enumerate(path.discrete):
                    # Move to the first point of the path (travel move)
                    first_point = path_coords[0]
                    f.write("G1 X{:.2f} Y{:.2f} F{}\n".format(
                        first_point[0], first_point[1], travel_speed * 60))
                    
                    # Extrusion moves along the path
                    for point in path_coords[1:]:
                        f.write("G1 X{:.2f} Y{:.2f} F{}\n".format(
                            point[0], point[1], print_speed * 60))
                
                f.write("\n")
        
        # End G-code
        f.write("; End of G-code\n")
        f.write("G1 Z10 F5000 ; Move nozzle up\n")
        f.write("G28 X0 Y0 ; Home X and Y axes\n")
        f.write("M104 S0 ; Turn off nozzle heater\n")
        f.write("M140 S0 ; Turn off bed heater\n")
        f.write("M84 ; Disable steppers\n")

if __name__ == "__main__":
    # Example usage
    input_stl = "example_model.stl"  # Replace with your STL file
    output_gcode = "output.gcode"
    
    slice_stl_to_gcode(
        stl_file=input_stl,
        output_file=output_gcode,
        layer_height=0.2,
        print_speed=60,
        travel_speed=120,
        nozzle_temp=200,
        bed_temp=60
    )
    
    print(f"G-code generated and saved to {output_gcode}")