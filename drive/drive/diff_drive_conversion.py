"""
Node for converting Twist velocity messages into
normalized wheel velocity messages
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# General imports
from . import diff_drive_kinematics as ddr
import sys
from signal import signal, SIGINT


class DiffDriveConversion(Node):
    """Declare node for translating twist velocities to wheel velocities"""

    # CONSTANTS
    PRINT_TOLERANCE = 0.0001

     # Quit program safely
    def quit_program_safely(self):

        print("\nExited Safely")
        sys.exit(0)

    # Callback for Ctrl+C
    def signalHandler(self, signal, frame):
        self.quit_program_safely()

    def __init__(self):

        # Signal handler for Ctrl+C
        signal(SIGINT, self.signalHandler)

        super().__init__('diff_drive_conversion')

        self.last_vel = (0.0, 0.0)
        self.subscription = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.message_callback,
            10)

        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/drive/analog_control', 10)

    def message_callback(self, ddr_msg):
        """This callback definition simply prints an info message to the
        console, along with the data it received."""

        vel_msg = Float64MultiArray()
        
        angular_vels = ddr.i_kinematics(
            ddr_msg.linear.x, ddr_msg.angular.z, 0.38735 * 2, 0.194)
        vel_msg.data = [ddr.angular_to_linear(vel) for vel in angular_vels]

        self.publisher_.publish(vel_msg)

        if self.last_vel != (vel_msg.data[0], vel_msg.data[1]):
            l_vel = vel_msg.data[0] if abs(vel_msg.data[0]) > self.PRINT_TOLERANCE else 0.0
            r_vel = vel_msg.data[1] if abs(vel_msg.data[1]) > self.PRINT_TOLERANCE else 0.0
            print(f'== COMMANDING VELOCITY [L: {l_vel} 😤 R: {r_vel}] ==')

        self.last_vel = (vel_msg.data[0], vel_msg.data[1])


def main(args=None):
    """run differential drive conversion node"""
    rclpy.init(args=args)

    minimal_subscriber = DiffDriveConversion()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
