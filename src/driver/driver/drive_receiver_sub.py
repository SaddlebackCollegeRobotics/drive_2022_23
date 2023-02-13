# ========================================================================================
# Author:   Jasper Doan     @wluxie
# Date:     02/01/2023
# ========================================================================================
# Description:
#   This is the ROS2 node that subscribes to the ROS2 topic 'controls', it first calibrates
#       the odrives, then set the velocity spin of the motors to the value based on the
#       controller input.
#
#   Note:
#       The values are normalized to be between -1 and 1, deadzoned to be 0 if the value is
#       less than AXIS_DEADZONE, and are rounded to 2 decimal places
# ========================================================================================
# Dependencies:
#   - ROS2 Foxy
#   - Python 3.8
#   - rclpy
# ========================================================================================


import rclpy                                # ROS2 Python API
from rclpy.node import Node                 # ROS2 Node API
from .calibrate import *                    # Odrive calibration API by Max Rehm, and Matin Qurbanzadeh
from std_msgs.msg import Float64MultiArray  # ROS2 Float64MultiArray message type


MIN_SPEED = -35     # Max negative velocity (ik bad naming convention)
MAX_SPEED = 35      # Max positive velocity
CREMENT = 5         # Speed increment/decrement value



# ==== ROS2 Subscriber Node ==============================================================
# Brief Description:
#   When run, this node will take in your analog/joystick controller input and publish it to
#   the ROS2 topic 'controls' as a Float64MultiArray. We used PS4 during development, but
#   this should work with any controller that has a left stick, right stick, and 2 triggers.
#
# Subscribe:
#   - msg :: Float64MultiArray[6]
#      + msg.data[0] :: left_stick_x        + msg.data[1] :: left_stick_y
#      + msg.data[2] :: right_stick_x       + msg.data[3] :: right_stick_y
#      + msg.data[4] :: left_trigger        + msg.data[5] :: right_trigger
#
# Run in Terminal:
#   source /opt/ros/foxy/setup.bash             <------ Source ROS2 Foxy environment (if not already sourced)
#   cd ~/drive_2022_23/src/driver/              <------ Navigate to driver package directory
#
#       colcon build --symlink-install          <------ Build driver package (if not already built)
#       . install/setup.bash                    <------ Source driver package environment
#       ros2 run driver drive_receiver_sub      <------ Run node
#
#       - or - 
#
#       make                                    <------ Make file that builts for you
#       make sub                                <------ Make file that sources and runs for you
# ========================================================================================
class DriveReceiverSub(Node):
    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Constructor
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def __init__(self, odrv_0, odrv_1=None):
        super().__init__('drive_receiver_sub')

        self.subscription = self.create_subscription(           # Create subscription to 'controls' topic
            Float64MultiArray,                                  # Message type
            'controls',                                         # Topic name
            self.listener_callback,                             # Callback function to call when message is received
            10)                                                 # Queue size
        self.subscription                                       # Prevent unused variable warning

        self.odrv_0 = odrv_0
        self.odrv_1 = odrv_1

        self.odrv0 = None
        self.odrv1 = None

        self.speed = MAX_SPEED # Test for now

    
    def close_loop_control(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


    def idle_state(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
        self.odrv1.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv1.axis1.requested_state = AXIS_STATE_IDLE


    def set_motor_vel(self, vel):
        self.odrv0.axis0.controller.input_vel = vel
        self.odrv0.axis1.controller.input_vel = vel
        self.odrv1.axis0.controller.input_vel = vel
        self.odrv1.axis1.controller.input_vel = vel

    
    def set_steer_vel(self, left, right):
        self.odrv0.axis0.controller.input_vel = left
        self.odrv0.axis1.controller.input_vel = left
        self.odrv1.axis0.controller.input_vel = right
        self.odrv1.axis1.controller.input_vel = right


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Listener Callback
    #       Called when a message is received on the 'controls' topic
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def listener_callback(self, msg):
        (ls_x, ls_y, rs_x, rs_y, l2, r2) = msg.data             # Unpack message data
        
        self.odrv0 = odrive.find_any(serial_number=self.odrv_0)       # Get odrive object
        if self.odrv_1:
            self.odrv1 = odrive.find_any(serial_number=self.odrv_1)   # Get odrive object


        fwd_vel = ls_y * self.speed
        turn_vel = abs(rs_x) * self.speed * 0.8

        idle = (ls_y == 0 and ls_x == 0)
        hold_left_stick = l2 > 0 and r2 == 0
        hold_right_stick = l2 == 0 and r2 > 0

        rndS = lambda x: round(x, 2)                # Formatting stick values


        print("😫😫 Left Stick:", (rndS(ls_x), rndS(ls_y)), "\tRight Stick:", (rndS(rs_x), rndS(rs_y)), "😫😫")
        print("😫😫 Left Speed:", rndS(fwd_vel), "\t\tRight Speed:", rndS(fwd_vel), "😫😫")
        

        # ==== Forward/Backward Movement ===========================
        if hold_left_stick and (not idle) and self.odrv_1:
            left_motor = -fwd_vel
            right_motor = fwd_vel

            if rs_x > 0:
                right_motor = right_motor * 0.5
            elif rs_x < 0:
                left_motor = left_motor * 0.5

            self.close_loop_control()
            self.set_steer_vel(left_motor, right_motor)
                

        # ==== Turn =================================================
        if hold_right_stick and (not idle) and self.odrv_1:
            self.close_loop_control()
            if rs_x > 0:
                self.set_motor_vel(-turn_vel)
            elif rs_x < 0:
                self.set_motor_vel(turn_vel)
            

        # ==== Stop all motors if no analog input ===================
        if idle and self.odrv_1:
            self.set_motor_vel(0)
            self.idle_state()

    



def main(args=None):
    odrives = get_all_odrives()
    odrv0 = odrives[0]
    odrv1 = odrives[1]

    calibrate_all_motors(odrv0, odrv1)


    rclpy.init(args=args)

    drive_receiver_sub = DriveReceiverSub(odrv0, odrv1)

    rclpy.spin(drive_receiver_sub)

    drive_receiver_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()