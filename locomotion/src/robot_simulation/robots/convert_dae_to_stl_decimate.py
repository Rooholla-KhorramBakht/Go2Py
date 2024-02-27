# Set the directory containing DAE files
dae_directory = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/b1_description/meshes"

# Set the output directory for STL files
stl_directory = "/home/meshin/dev/quadruped/xterra/quadruped_locomotion/src/robots/b1_description/meshes"

# Set the ratio for the decimation modifier (adjust as needed)
decimation_ratio = 0.1

# List all files in the directory
files = os.listdir(dae_directory)

# Filter out only DAE files
dae_files = [file for file in files if file.endswith(".dae")]

# Loop through each DAE file and convert to STL
for dae_file in dae_files:
    # Load DAE file
    bpy.ops.wm.collada_import(filepath=os.path.join(dae_directory, dae_file))

    # Apply the decimation modifier
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.modifier_add(type='DECIMATE')
    
    # Explicitly set the context for the modifier_apply operation
#    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    
    bpy.context.object.modifiers["Decimate"].ratio = decimation_ratio
    bpy.ops.object.modifier_apply(modifier="Decimate")

    # Set the output path for STL file
    stl_file = os.path.join(stl_directory, dae_file.replace(".dae", ".stl"))

    # Export as STL with use_mesh_modifiers set to False
    bpy.ops.export_mesh.stl(filepath=stl_file, use_selection=True, use_mesh_modifiers=False, ascii=False)

    # Remove the imported mesh to avoid overlap
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

# Print a message indicating the conversion is complete
print("DAE to STL conversion completed.")