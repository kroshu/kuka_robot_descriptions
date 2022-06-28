import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_file(absolute_file_path):
    # package_path = get_package_share_directory(package_name)
    # absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    yaml_file_name = get_package_share_directory('urdflbriiwa7') + "/config/urdflbriiwa7_control.yaml"

    robot_description_config = load_file(get_package_share_directory('urdflbriiwa7') + "urdf/urdflbriiwa7.urdf")
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file(get_package_share_directory('urdflbriiwa7') + '/urdf/urdflbriiwa7.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}


    # RViz
    rviz_config_file = get_package_share_directory('urdflbriiwa7') + "/launch/urdf.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'URDFLBRiiwa7RobotBase'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    # Joint state publisher
    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 arguments=[get_package_share_directory(
                                     'urdflbriiwa7') + '/urdf/urdflbriiwa7.urdf'],
                                 output='log',
                                 parameters=[{'source_list': ['/reference_joint_state']}])
                                 



    return LaunchDescription([ static_tf, robot_state_publisher, joint_state_publisher, rviz_node,])
