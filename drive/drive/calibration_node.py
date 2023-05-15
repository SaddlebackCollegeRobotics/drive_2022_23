# Author: Cameron Rosenthal - @Supernova1114

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .calibrate import get_all_odrives, calibrate_all_motors

# General imports
import sys
from signal import signal, SIGINT


class Calibration_Node(Node):

    # Quit program safely
    def quit_program_safely(self):

        print("\nExited Safely")
        sys.exit(0)


    # Callback for Ctrl+C
    def signalHandler(self, signal, frame):
        self.quit_program_safely()


    # Constructor
    def __init__(self):

        # Signal handler for Ctrl+C
        signal(SIGINT, self.signalHandler)

        # Give the node a name.
        super().__init__('drive_calibration_node')

        odrives = get_all_odrives() # Get ODrive serial numbers using linux lsusb
        odriveCount = len(odrives)

        # Check if ODrives are connected
        if odriveCount == 0 or odriveCount == 1:
            print("Not all ODrives connected. Exiting...")
        else:
            print("Calibrating ODrives...")
            calibrate_all_motors(odrives[0], odrives[1])
            
        self.quit_program_safely()




def main(args=None):
    rclpy.init(args=args)

    calibration_node = Calibration_Node()

    rclpy.spin(calibration_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calibration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()