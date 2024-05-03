import xml.etree.ElementTree as ET
import sys
import bpy
import os
from mathutils import Vector


def remove_tags(urdf_file):
    # Parse the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Get the absolute path of the URDF file
    abs_path = os.path.abspath(urdf_file)
    index = abs_path.rfind('kuka_robot_descriptions')
    workspace = abs_path[:index + len('kuka_robot_descriptions')]

    # Find the ros2_control tag and remove it
    for ros2_control in root.findall('ros2_control'):
        root.remove(ros2_control)

    # Find all link tags
    for link in root.findall('.//link'):
        # Find the visual tag
        visual_tag = link.find('visual')

        # Skip links without visual tag
        if visual_tag is None:
            continue
        # Find the mesh tag within the visual tag
        mesh_tag = visual_tag.find('.//mesh')
        # Get the filename attribute
        filename = workspace + "/src/kuka_robot_descriptions/" + mesh_tag.get('filename').replace("package://", "", 1)
        print(filename)
        # Find the origin tag within the visual tag
        origin_tag = visual_tag.find('.//origin')

        # Get the coordinates
        xyz = origin_tag.get('xyz')
        rpy = origin_tag.get('rpy')

        # Remove the visual attribute
        link.remove(visual_tag)

        origin, size = calc_bounding_box(filename, xyz, rpy)

        collision_tag = visual_tag = link.find('collision')
        geometry_tag = collision_tag.find('geometry')
        # Find the mesh tag within the geometry tag
        mesh_tag = geometry_tag.find('mesh')
        # Create a new box tag
        box_tag = ET.Element('box')
        # Set the size attribute of the box tag
        box_tag.set('size', ' '.join(map(str, size)))
        # Replace the mesh tag with the box tag
        index = list(geometry_tag).index(mesh_tag)
        geometry_tag[index] = box_tag

        # TODO: modify origin tag

    # Write the modified tree back to the file
    ET.indent(tree, space="  ", level=0)
    tree.write(urdf_file, encoding="utf-8")


def calc_bounding_box(file_path, xyz, rpy):
    # Delete all mesh objects
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

    # Get the file extension
    _, file_extension = os.path.splitext(file_path)

    # Import the file based on its extension
    if file_extension.lower() == ".stl":
        bpy.ops.import_mesh.stl(filepath=file_path)
    elif file_extension.lower() == ".dae":
        bpy.ops.wm.collada_import(filepath=file_path, auto_connect=True)
    else:
        print(f"Unsupported file extension: {file_extension}")
        return

    # Get the imported object
    obj = bpy.context.selected_objects[0]

    # If the object is an EMPTY, look for MESH objects among its children
    if obj.type == 'EMPTY':
        meshes = [child for child in obj.children if child.type == 'MESH']

        # If there are multiple meshes, join them into one
        if len(meshes) > 1:
            bpy.ops.object.select_all(action='DESELECT')
            for mesh in meshes:
                mesh.select_set(True)
            bpy.context.view_layer.objects.active = meshes[0]
            bpy.ops.object.join()

        # The joined mesh is now the active object
        obj = bpy.context.active_object

    # Calculate the bounding box
    bbox_corners = [obj.matrix_world @ Vector(v[:]) for v in obj.bound_box]

    # Calculate the center of the bounding box
    bbox_center = sum(bbox_corners, Vector()) / len(bbox_corners)

    # Calculate the size of the bounding box
    bbox_size = [max(corner[i] for corner in bbox_corners) - min(corner[i] for corner in bbox_corners) for i in range(3)]

    return bbox_center, bbox_size


if __name__ == "__main__":
    # Check if a file name was provided
    if len(sys.argv) < 2:
        print("Usage: python modify_urdf.py your_file.urdf")
        sys.exit(1)

    # Call the function with your URDF file
    remove_tags(sys.argv[1])
