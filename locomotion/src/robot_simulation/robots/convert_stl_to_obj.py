import bpy
import os

# Set the directory containing STL files
directory = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/m2_description/meshes"

# Set the output directory for OBJ files
output_directory = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/m2_description/mujoco/assets"

# List all files in the directory
files = os.listdir(directory)

# Filter out only STL files
stl_files = [file for file in files if file.endswith(".stl") or file.endswith(".STL")]

# Loop through each STL file and convert to OBJ
for stl_file in stl_files:
    # Load STL file
    bpy.ops.import_mesh.stl(filepath=os.path.join(directory, stl_file))

    # Set the output path for OBJ file
    obj_file = os.path.join(output_directory, stl_file.replace(".STL", ".obj"))

    # Export as OBJ
    bpy.ops.wm.obj_export(
        filepath=obj_file,
        check_existing=True,
    )

    # Remove the imported mesh to avoid overlap
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

# Print a message indicating the conversion is complete
print("STL to OBJ conversion completed.")