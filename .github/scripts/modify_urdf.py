import xml.etree.ElementTree as ET
import sys

def remove_tags(urdf_file):
    # Parse the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    # Find the ros2_control tag and remove it
    for ros2_control in root.findall('ros2_control'):
        root.remove(ros2_control)

    # Find all link tags
    for link in root.findall('.//link'):
        # Find all visual tags within the current link tag and remove them
        for visual in link.findall('visual'):
            link.remove(visual)

    # Write the modified tree back to the file
    tree.write(urdf_file)

if __name__ == "__main__":
    # Check if a file name was provided
    if len(sys.argv) < 2:
        print("Usage: python modify_urdf.py your_file.urdf")
        sys.exit(1)

    # Call the function with your URDF file
    remove_tags(sys.argv[1])
