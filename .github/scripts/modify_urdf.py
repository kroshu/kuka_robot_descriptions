# Copyright 2024 Aron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import xml.etree.ElementTree as ET
import sys
import bpy
import os
from mathutils import Vector, Matrix
from ament_index_python.packages import get_package_share_directory


def remove_tags(urdf_file):
    # Parse the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Find the ros2_control tag and remove it
    for ros2_control in root.findall("ros2_control"):
        root.remove(ros2_control)

    # Find all link tags
    for link in root.findall(".//link"):
        # Find the visual tag
        visual_tag = link.find("visual")

        # Skip links without visual tag
        if visual_tag is None:
            continue
        # Find the mesh tag within the visual tag
        mesh_tag = visual_tag.find(".//mesh")
        # Get the filename attribute
        # Extract the package name and relative file path from the input string
        package_name, relative_file_path = mesh_tag.get("filename").split("://")[1].split("/", 1)

        # Get the package share directory
        package_share_directory = get_package_share_directory(package_name)

        # Combine the package share directory with the relative file path to get the absolute path
        mesh_file = os.path.join(package_share_directory, relative_file_path)
        # Find the origin tag within the visual tag
        origin_tag = visual_tag.find(".//origin")

        # Get the coordinates
        xyz = Vector(list(map(float, origin_tag.get("xyz").split())))
        rpy = Vector(list(map(float, origin_tag.get("rpy").split())))

        # Remove the visual attribute
        link.remove(visual_tag)

        origin, size = calc_bounding_box(mesh_file, xyz, rpy)

        # Set modified xyz for collision origin
        # Orientation is not modified, as box is always aligned to original frame
        collision_tag = link.find("collision")
        coll_origin = collision_tag.find(".//origin")
        coll_origin.set("xyz", " ".join(map(str, origin)))
        coll_origin.set("rpy", " ".join(map(str, rpy)))

        geometry_tag = collision_tag.find("geometry")
        # Find the mesh tag within the geometry tag
        mesh_tag = geometry_tag.find("mesh")
        # Create a new box tag
        box_tag = ET.Element("box")
        # Set the size attribute of the box tag
        box_tag.set("size", " ".join(map(str, size)))
        # Replace the mesh tag with the box tag
        index = list(geometry_tag).index(mesh_tag)
        geometry_tag[index] = box_tag

    # Write the modified tree back to the file
    ET.indent(tree, space="  ", level=0)
    tree.write(urdf_file, encoding="utf-8")


def calc_bounding_box(file_path, xyz, rpy):
    # Delete all mesh objects
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.select_by_type(type="MESH")
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
    if obj.type == "EMPTY":
        meshes = [child for child in obj.children if child.type == "MESH"]

        # If there are multiple meshes, join them into one
        if len(meshes) > 1:
            bpy.ops.object.select_all(action="DESELECT")
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
    bbox_size = [
        max(corner[i] for corner in bbox_corners) - min(corner[i] for corner in bbox_corners)
        for i in range(3)
    ]

    # Convert the original rotation from RPY to a rotation matrix
    # roll-pitch-yaw is extrinsic Euler rotation in x-y-z order
    # -> R = R(Z)*R(Y)*R(X)
    rotation_matrix = (
        Matrix.Rotation(rpy.z, 3, "Z")
        @ Matrix.Rotation(rpy.y, 3, "Y")
        @ Matrix.Rotation(rpy.x, 3, "X")
    )

    # Rotate the new translation by the original rotation
    rotated_new_translation = rotation_matrix @ bbox_center

    # Add the rotated new translation to the original translation
    bbox_center = xyz + rotated_new_translation
    return bbox_center, bbox_size


if __name__ == "__main__":
    # Check if a file name was provided
    if len(sys.argv) < 2:
        print("Usage: python modify_urdf.py your_file.urdf")
        sys.exit(1)

    # Call the function with your URDF file
    remove_tags(sys.argv[1])
