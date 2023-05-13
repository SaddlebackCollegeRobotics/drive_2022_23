"""
Launch file for starting drive subsystem on rover.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for rover drive system.

    Launches conversion from twist messages to normalized wheel velocities,
    along with motor drivers.
    """
    diff_drive_conversion = Node(
        package='drive',
        executable='diff_drive_conversion',
        output='screen',
    )

    motor_driver = Node(
        package='drive',
        executable='motor_driver',
        output='screen',
    )

    return LaunchDescription([
        diff_drive_conversion,
        motor_driver,
    ])
