from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    omnimove_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [FindPackageShare("kuka_omnimove_e575_support"), "/launch", "/omnimove_launch.py"]))
    quantec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("kuka_kss_rsi_driver"), "/launch", "/startup.launch.py"]),
        launch_arguments={'robot_model': 'kr210_r2700_2', 'robot_family': 'quantec'}.items())
    transform_publisher = LaunchDescription([Node(package='tf2_ros', executable='static_transform_publisher',
                                                  arguments=[
                                                      '--x', '0.89567',  '--y', '0.00247', '--z', '0.48018', '--frame-id', 'omnimove_base', '--child-frame-id', 'base_link'])])
    return LaunchDescription([omnimove_launch, quantec_launch, transform_publisher])
