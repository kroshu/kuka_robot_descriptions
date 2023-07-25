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
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_omnimove_e575_support"),
                    "urdf",
                    "omnimove_with_arm.xacro",
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

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kuka_omnimove_e575_support"), "rviz", "rviz.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_description_path = (get_package_share_path('kuka_omnimove_e575_support') / 'urdf' /
                              'omnimove_with_arm.xacro')


    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', str(robot_description_path)]), value_type=str
        )}],
        #remappings=[("joint_states", "rsi_joint_state")]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_file)],
    )

    joint_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

#    robot_controller_spawner = Node(
#        package="controller_manager",
#        executable="spawner",
#        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
#    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_trajectory_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
#    delay_robot_controller_spawner_after_joint_trajectory_spawner = RegisterEventHandler(
#        event_handler=OnProcessExit(
#            target_action=joint_trajectory_spawner,
#            on_exit=[robot_controller_spawner],
#        )
#    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_trajectory_spawner,
        delay_rviz_after_joint_trajectory_spawner,
        joint_broadcaster_spawner,
#        delay_robot_controller_spawner_after_joint_trajectory_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
