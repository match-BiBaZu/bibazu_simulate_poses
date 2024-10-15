import bpy

def convert_stl_to_obj(stl_filepath, obj_filepath):
    # Clear existing mesh data
    bpy.ops.wm.read_factory_settings(use_empty=True)
    
    # Import STL file
    bpy.ops.import_mesh.stl(filepath=stl_filepath)
    
    # Export as OBJ file
    bpy.ops.export_scene.obj(filepath=obj_filepath)

    if __name__ == "__main__":
        stl_filepath = "/path/to/your/input_file.stl"
        obj_filepath = "/path/to/your/output_file.obj"
        convert_stl_to_obj(stl_filepath, obj_filepath)