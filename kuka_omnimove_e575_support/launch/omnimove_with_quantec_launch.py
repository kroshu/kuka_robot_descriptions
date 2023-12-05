from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
  omnimove_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("kuka_omnimove_e575_support"), "/launch", "/omnimove_launch.py"]))
  quantec_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("kuka_rsi_hw_interface"), "/launch", "/startup.launch.py"]), launch_arguments={'robot_model': 'kr210_r2700-2'}.items())
  return LaunchDescription([omnimove_launch, quantec_launch])
