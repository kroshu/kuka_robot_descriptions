import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def load_file(absolute_file_path):
    # package_path = get_package_share_directory(package_name)
    # absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    gui = LaunchConfiguration('gui', default='True')

    robot_description_path = (get_package_share_directory('kuka_lbr_iiwa7_support') +
                              "/urdf/lbriiwa7.xacro")
    robot_description = {'robot_description': ParameterValue(
            Command(['xacro ', str(robot_description_path)]), value_type=str
        )}

    # RViz
    rviz_config_file = get_package_share_directory('kuka_lbr_iiwa7_support') + "/launch/urdf.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'LBRiiwa7RobotBase'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 remappings=[
                                     ("joint_states", "lbriiwa7/joint_states")
                                 ],
                                 parameters=[robot_description])
                                 
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                 executable='joint_state_publisher_gui',
                                 name='joint_state_publisher_gui',
                                 output='both',
                                 remappings=[
                                     ("joint_states", "lbriiwa7/joint_states")
                                 ],
                                 parameters=[robot_description])

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'gui': gui}.items(),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'lbriiwa7', '-topic', '/robot_description'],
                        output='screen')

    return LaunchDescription([static_tf, robot_state_publisher, joint_state_publisher_gui, gazebo_launch, spawn_entity, rviz_node])
   