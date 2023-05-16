"""
Launch file for starting drive subsystem on rover.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """
    Generate launch description for rover drive system.

    Launches conversion from twist messages to normalized wheel velocities,
    along with motor drivers.
    """

    calibrate = LaunchConfiguration('calibrate')

    diff_drive_conversion = Node(
        package='drive',
        executable='diff_drive_conversion',
        output='screen',
        condition=UnlessCondition(calibrate)
    )

    calibration_node = Node(
        package='drive',
        executable='calibration',
        output='screen',
        condition=IfCondition(calibrate),
    )

    motor_driver = Node(
        package='drive',
        executable='motor_driver',
        output='screen',
        condition=UnlessCondition(calibrate)
    )


    return LaunchDescription([
        
        DeclareLaunchArgument(
            'calibrate',
            default_value='false',
            description='True to calibrate motors'),
        
        diff_drive_conversion,
        calibration_node,
        motor_driver,
    ])