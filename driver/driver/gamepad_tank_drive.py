""" ======================================================================================
# Description:
#   This is the ROS2 node that publishes the controller input to the ROS2 topic 
#   '/diff_cont/cmd_vel_unstamped', type geometry_msgs Twist
#    linear:
#       x: float
#       y: 0
#       z: 0
#   angular:
#       x: 0
#       y: 0
#       z: float
#           
#   Note:
#       The values are normalized to be between -1 and 1, deadzoned to be 0 if the value is 
#       less than AXIS_DEADZONE, and are rounded to 2 decimal places
#       The values are published at a rate of 10 Hz
# ========================================================================================
# Dependencies:
#   - ROS2 Humble
# ==================================================================================== """


import rclpy                                # ROS2 Python API
from rclpy.node import Node                 # ROS2 Node API
from . import gamepad_input as gmi          # Gamepad input API 
from . import diff_drive_kinematics as ddr  # Differential drive robot kinematics 
from .buttons import Buttons                # Gamepad button callbacks
from std_msgs.msg import Float64MultiArray  # ROS2 Float64MultiArray message type
from geometry_msgs.msg import Twist         # ROS2 control message
import os                                   # OS API


# ==== Controller Configuration ==========================================================
AXIS_DEADZONE = 0.1                                             # Deadzone is 0 to 1 | Note: axis value will be 0 until you move past the deadzone

gmi.setConfigFile(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../share/driver/gamepads.config'))

b = Buttons()                                                   # Create button callbacks object
connectionEvents = [b.onGamepadConnect, b.onGamepadDisconnect]  # Set connection callbacks



# ==== Differential Drive Robot Kinematics ===============================================
# Title: Motion Model for the Differential Drive Robot
# Authors: Frank Dellaert, Seth Hutchinson
# Date: 2021
# Availability: https://www.roboticsbook.org/S52_diffdrive_actions.html
# ========================================================================================
WHEEL_SEPARATION = 0.38735 * 2
WHEEL_RADIUS = 0.194
   
def ddr_ik(v_x: float, omega: float, L=WHEEL_SEPARATION, r=WHEEL_RADIUS) -> tuple[float, float]:
    """DDR inverse kinematics: calculate angular wheels speeds from desired velocity.
    returns: (left wheel angular velocity, right wheel angular velocity)"""
    return (v_x - (L/2)*omega)/r, (v_x + (L/2)*omega)/r  

def ddr_fk(phidot_L: float, phidot_R: float, L=WHEEL_SEPARATION, r=WHEEL_RADIUS) -> tuple[float, float]:
    """DDR forward kinematics: calculate desired velocity from angular wheels speeds.
    returns: (linear velocity, angular velocity)"""
    return(phidot_R+phidot_L)*r/2, (phidot_R-phidot_L)*r/L 



class GamepadDrive(Node):
    """ 
    When run, this node will take in your analog/joystick controller input and publish 
    it to the ROS2 topic 'controls' as a Twist message. We used PS4 during 
    development, but this should work with any controller that has a left stick, right 
    stick, and 2 triggers.
     
    Publishes:
        geometry_msg :: Twist -- linear and angular movement commands for differential drive
    """

    # CONSTANTS
    TIMER_PERIOD = 0.1
    PRINT_TOLERANCE = 0.0001

    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Constructor
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def __init__(self):
        super().__init__('gamepad_tank_drive')       
        self.last_l_analog = -1
        self.last_r_analog = -1
                                               # Create node with name 'controller_pub'

        # ros2_controller listens for twist messages on /diff_cont/cmd_vel_unstamped
        # Publishing                            [type]  [topic]                        [queue_size]
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)  
        self.timer = self.create_timer(GamepadDrive.TIMER_PERIOD, self.timer_callback)             # Create timer to publish controller input

        # Note: I disabled button callbacks because they were causing the program to crash
        #       I think it was because the callbacks were trying to access the ROS2 node
        #       while the node was being spinned. I'm not sure, but I'll look into it
        #       later. For now, we can just use the controller input to drive the rover
        gmi.run_event_loop(None, None, None, connectionEvents)                          # Async loop to handle gamepad button events


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Timer Callback
    #       This function is called every time the timer goes off. It gets the controller
    #       input and publishes it to the ROS2 topic 'controls'
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def timer_callback(self):
        msg = Float64MultiArray()                           # Create message
        msg = Twist()  # create message

        # Twist message:
        # linear:
        #     x: 0.0
        #     y: 0.0
        #     z: 0.0
        # angular:
        #     x: 0.0
        #     y: 0.0
        #     z: 0.0

        gp = gmi.getGamepad(0)
        ls_x, ls_y = gmi.getLeftStick(gp, AXIS_DEADZONE)  # Get left stick
        rs_x, rs_y = gmi.getRightStick(gp, AXIS_DEADZONE) # Get right stick
        l2, r2 = gmi.getTriggers(gp, AXIS_DEADZONE)       # Get triggers
        l1, r1 = gmi.getButtonValue(gp, 7), gmi.getButtonValue(gp, 8)
    
        enable_left = l2 > 0
        enable_right = r2 > 0

        l_analog = float(-ls_y) if enable_left else 0.0
        r_analog = float(-rs_y) if enable_right else 0.0

        # calculate linear and angular movement using differential drive forward kinematics
        # using transformation equation v=rω, ω=v/r
        msg.linear.x, msg.angular.z  = ddr.f_kinematics(
            ddr.linear_to_angular(l_analog), ddr.linear_to_angular(r_analog)
        )        
        
        if self.last_l_analog != l_analog or self.last_r_analog != r_analog: 
            linear = msg.linear.x if abs(msg.linear.x) > self.PRINT_TOLERANCE else 0.0
            angular = msg.angular.z if abs(msg.angular.z) > self.PRINT_TOLERANCE else 0.0
            print(f'== SENDING [Linear: ${linear:.3} 😤 Angular: ${angular:.3}  ==')

        self.last_l_analog, self.last_r_analog = l_analog, r_analog

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    controller_pub = GamepadDrive()

    rclpy.spin(controller_pub)

    controller_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()