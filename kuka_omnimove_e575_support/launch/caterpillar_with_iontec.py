from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
  omnimove_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("kuka_omnimove_e575_support"), "/launch", "/caterpillar_launch.py"]))
  iontec_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("kuka_kss_rsi_driver"), "/launch", "/startup.launch.py"]))
  transform_publisher = LaunchDescription([Node(package='tf2_ros', executable='static_transform_publisher',
                                                arguments=[
                                                  '--x', '0.559', '--y', '0.00', '--z', '0.674', '--frame-id',
                                                  'caterpillar_base', '--child-frame-id', 'base_link'])])
  return LaunchDescription([omnimove_launch, iontec_launch, transform_publisher])
