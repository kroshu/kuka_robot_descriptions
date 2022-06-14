import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node

def generate_launch_description():
    robot_description_path = get_package_share_path('kuka_kr6_support') / 'urdf' / 'kr6r700sixx.xacro'
    rviz_config_path = get_package_share_path('kuka_kr6_support') / 'rviz' / 'rviz.rviz'
    # robot_description = open(robot_description_path).read()

    robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', str(robot_description_path)]), value_type=str
        )}]
    )

    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{'use_gui': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_path)],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])


# <?xml version="1.0"?>
# <launch>
#   <param name="robot_description" command="$(find xacro)/xacro '$(find kuka_kr6_support)/urdf/kr6r700sixx.xacro'"/>
#   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
#     <param name="use_gui" value="true" />
#   </node>
#   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
#   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find industrial_robot_client)/config/robot_state_visualize.rviz" required="true" />
# </launch>
