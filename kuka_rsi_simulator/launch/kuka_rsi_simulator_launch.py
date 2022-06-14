from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    rsi_ip_address_ = DeclareLaunchArgument(
        'rsi_ip_address_', default_value=TextSubstitution(text='127.0.0.1')
    )
    rsi_port_ = DeclareLaunchArgument(
        'rsi_port_', default_value=TextSubstitution(text='59152')
    )

    return LaunchDescription([
        rsi_ip_address_,
        rsi_port_,
        Node(
            package='kuka_rsi_simulator',
            executable='rsi_simulator',
            name='kuka_rsi_simulator',
            parameters=[{
                'rsi_ip_address_': LaunchConfiguration('rsi_ip_address_'),
                'rsi_port_': LaunchConfiguration('rsi_port_'),
            }]
        )
    ])