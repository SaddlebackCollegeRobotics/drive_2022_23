import rclpy
from rclpy.node import Node

from . import diff_drive_kinematics as ddr

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray  



class DiffDriveConversion(Node):

    def __init__(self):

        super().__init__('diff_drive_conversion')

        self.last_vel = (0.0, 0.0)

        self.subscription = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.message_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Float64MultiArray, '/drive/analog_control', 10)       


    # This callback definition simply prints an info message to the console, along with the data it received. 
    def message_callback(self, ddr_msg):
        vel_msg = Float64MultiArray()
        
        angular_vels = ddr.i_kinematics(ddr_msg.linear.x, ddr_msg.angular.z) 
        vel_msg.data = [ddr.angular_to_linear(vel) for vel in angular_vels]
        
        self.publisher_.publish(vel_msg)


        if self.last_vel != (vel_msg.data[0], vel_msg.data[1]):
            print(f'== COMMANDING VELOCITY [L: {vel_msg.data[0]} 😤 R: {vel_msg.data[1]}] ==');

        self.last_vel = (vel_msg.data[0], vel_msg.data[1])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DiffDriveConversion()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
