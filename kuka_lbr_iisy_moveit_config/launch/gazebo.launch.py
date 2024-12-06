import os
import yaml
from moveit_configs_utils import MoveItConfigsBuilder
import launch_ros.descriptions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    dof = LaunchConfiguration("dof")
    robot_urdf_folder = LaunchConfiguration("robot_urdf_folder")    
    robot_urdf_filepath = LaunchConfiguration("robot_urdf_filepath")    
    robot_srdf_folder = LaunchConfiguration("robot_srdf_folder")        
    robot_srdf_filepath = LaunchConfiguration("robot_srdf_filepath")
    robot_kinematics_folder = LaunchConfiguration("robot_kinematics_folder")
    
    moveit_config = (
        MoveItConfigsBuilder("kuka_lbr_iisy")
        .robot_description(file_path=get_package_share_directory(robot_urdf_folder.perform(context)) + robot_urdf_filepath.perform(context))
        .robot_description_semantic(get_package_share_directory(robot_srdf_folder.perform(context)) + robot_srdf_filepath.perform(context))
        .robot_description_kinematics(file_path=f"{get_package_share_directory(robot_kinematics_folder.perform(context))}/config/kinematics.yaml")
        .trajectory_execution(file_path=f"{get_package_share_directory(robot_kinematics_folder.perform(context))}/config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .joint_limits(file_path=get_package_share_directory("kuka_lbr_iisy_support") + f"/config/{robot_model.perform(context)}_joint_limits.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines("ompl", ["ompl"])
        .to_moveit_configs()
    )
    
    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    ],
    )

    rviz_config_file = (get_package_share_directory("kuka_resources") + f"/config/planning_{dof.perform(context)}_axis.rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(robot_urdf_folder.perform(context)),
                    robot_urdf_filepath.perform(context).split("/")[1],
                    robot_urdf_filepath.perform(context).split("/")[2],
                ]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    controller_config = (get_package_share_directory("kuka_resources") + f"/config/fake_hardware_config_{dof.perform(context)}_axis.yaml")

    controller_manager_node = "/controller_manager"

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
    )

    # Spawn controllers
    def controller_spawner(controller_with_config):
        arg_list = [controller_with_config[0], "-c", controller_manager_node]
        if controller_with_config[1] is not None:
            arg_list.append("-p")
            arg_list.append(controller_with_config[1])
        return Node(package="controller_manager", executable="spawner", arguments=arg_list)

    controller_names_and_config = [
        ("joint_state_broadcaster", None),
        ("joint_trajectory_controller", controller_config),
    ]

    controller_spawners = [
        controller_spawner(controllers) for controllers in controller_names_and_config
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
                    {"use_sim_time": True},
                    ],
        )
    
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), 'launch', 'gz_sim.launch.py')),
        launch_arguments={"gz_args": '-r empty.sdf -v 0',}.items(),
    )
    
    gazebo_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'topic': '/robot_description'}],
        output='screen'
    )
    
    gazebo_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[{'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',}],
        output='screen'
    )
    
    to_start = [move_group_server, gazebo_simulation, gazebo_spawner, gazebo_clock_bridge, robot_state_publisher, rviz] + controller_spawners
    
    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"))
    launch_arguments.append(DeclareLaunchArgument("dof", default_value="6"))
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_folder", default_value="kuka_lbr_iisy_support"))
    launch_arguments.append(DeclareLaunchArgument("robot_urdf_filepath", default_value=f"/urdf/lbr_iisy3_r760_gazebo.urdf.xacro"))
    launch_arguments.append(DeclareLaunchArgument("robot_srdf_folder", default_value="kuka_lbr_iisy_moveit_config"))
    launch_arguments.append(DeclareLaunchArgument("robot_srdf_filepath", default_value=f"/urdf/lbr_iisy3_r760.srdf"))
    launch_arguments.append(DeclareLaunchArgument("robot_kinematics_folder", default_value="kuka_lbr_iisy_moveit_config"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
