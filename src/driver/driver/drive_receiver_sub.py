import rclpy
from rclpy.node import Node
# from calibrate import *
import subprocess
from std_msgs.msg import Float64MultiArray


MIN_SPEED = -20
MAX_SPEED = 20
CREMENT = 5


class DriveReceiverSub(Node):
    def __init__(self):
        super().__init__('drive_receiver_sub')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'controls',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.speed = 5

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    
def main(args=None):
    rclpy.init(args=args)

    drive_receiver_sub = DriveReceiverSub()

    rclpy.spin(drive_receiver_sub)

    drive_receiver_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()