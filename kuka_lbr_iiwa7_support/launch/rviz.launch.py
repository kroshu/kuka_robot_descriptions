from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def load_file(absolute_file_path):
    # package_path = get_package_share_directory(package_name)
    # absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_path = (get_package_share_directory('kuka_lbr_iiwa7_support') +
                              "/urdf/lbriiwa7.xacro")
    robot_description = {'robot_description': ParameterValue(
            Command(['xacro ', str(robot_description_path)]), value_type=str
        )}

    robot_semantic_config = load_file(get_package_share_directory('kuka_lbr_iiwa7_support')
                                      + '/urdf/lbriiwa7.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_semantic_config}

    # RViz
    rviz_config_file = get_package_share_directory('kuka_lbr_iiwa7_support') + "/launch/urdf.rviz"
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
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world',
                                'LBRiiwa7RobotBase'])

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
                                 output='log',
                                 parameters=[{'source_list': ['/reference_joint_state']}])

    return LaunchDescription([static_tf, robot_state_publisher, joint_state_publisher, rviz_node])
