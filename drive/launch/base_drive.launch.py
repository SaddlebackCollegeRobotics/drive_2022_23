"""
Launch file for starting drive subsystem at base station.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for gamepad controller node
    Can specify controller to be of type diff_drive or tank_drive
    """
    use_diff_drive = LaunchConfiguration('use_diff_drive')

    diff_drive_controller = Node(
        package='drive',
        executable='gamepad_diff_drive',
        output='screen',
        condition=IfCondition(use_diff_drive)
    )

    tank_drive_controller = Node(
        package='drive',
        executable='gamepad_tank_drive',
        output='screen',
        condition=UnlessCondition(use_diff_drive)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_diff_drive',
            default_value='true',
            description='True for differential drive, \
                         otherwise tank drive'),

        diff_drive_controller,
        tank_drive_controller,
    ])
