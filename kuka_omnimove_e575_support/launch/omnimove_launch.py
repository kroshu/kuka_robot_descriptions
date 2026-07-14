from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path



def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_omnimove_e575_support"),
                    "urdf",
                    "omnimove_urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kuka_omnimove_e575_support"),
            "config",
            "omnimove_controller.yaml",
        ]
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        namespace="platform",
    )


    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='platform',
        parameters=[robot_description],
        #remappings=[("joint_states", "rsi_joint_state")]
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="platform",
        arguments=["joint_state_broadcaster", "--controller-manager", "/platform/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="platform",
        arguments=["velocity_command_controller", "--controller-manager", "/platform/controller_manager"],
    )


    nodes = [
        control_node,
        robot_state_pub_node,
        joint_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(nodes)
