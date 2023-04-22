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


# ==== ROS2 Subscriber Node ==============================================================
# Brief Description:
#   When run, this node will take in your analog/joystick controller input and publish it to
#   the ROS2 topic 'controls' as a Float64MultiArray. We used PS4 during development, but
#   this should work with any controller that has a left stick, right stick, and 2 triggers.
#
# Subscribe:
#   - msg :: Float64MultiArray[2]
#      + msg.data[0] :: l_analog          + msg.data[1] :: r_analog
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
    # CONSTANTS
    MIN_SPEED = -30                         # Max negative velocity (ik bad naming convention)
    MAX_SPEED = 30                          # Max positive velocity
    MIN_VOLTAGE = 11.4                      # Cut off battery voltage

    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Constructor
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def __init__(self, odrv_0=None, odrv_1=None):
        super().__init__('drive_receiver_sub')

        self.subscription = self.create_subscription(           # Create subscription to 'controls' topic
            Float64MultiArray,                                  # Message type
            'drive/analog_control',                             # Topic name
            self.listener_callback,                             # Callback function to call when message is received
            10)                                                 # Queue size

        self.odrv0 = odrive.find_any(serial_number=odrv_0)      # Get odrive object
        self.odrv1 = odrive.find_any(serial_number=odrv_1)      # Get odrive object

        self.vbus_voltage0 = self.odrv0.vbus_voltage
        self.vbus_voltage1 = self.odrv1.vbus_voltage

        self.speed = DriveReceiverSub.MAX_SPEED                 # Set speed to max speed



    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Voltage Check
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def voltage_check(self):
        self.vbus_voltage0 = self.odrv0.vbus_voltage
        self.vbus_voltage1 = self.odrv1.vbus_voltage

        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        if self.vbus_voltage0 <= DriveReceiverSub.MIN_VOLTAGE or self.vbus_voltage1 <= DriveReceiverSub.MIN_VOLTAGE:
            print("🛑🛑🛑 Battery is low, please charge battery 🛑🛑🛑")
            print("\tOdrive0 Voltage Reading: " + str(self.vbus_voltage0) + " V")
            print("\tOdrive1 Voltage Reading: " + str(self.vbus_voltage1) + " V")
        else:
            print("🔋🔋🔋 Battery is good, you may drive 🔋🔋🔋")
            print("\tOdrive0 Voltage Reading: " + str(self.vbus_voltage0) + " V")
            print("\tOdrive1 Voltage Reading: " + str(self.vbus_voltage1) + " V")


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Set closed loop control
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,     
    def close_loop_control(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Set idle state
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def idle_state(self):
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
        self.odrv1.axis0.requested_state = AXIS_STATE_IDLE
        self.odrv1.axis1.requested_state = AXIS_STATE_IDLE


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Set motor speed
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,, 
    def set_motor_velocity(self, left_speed, right_speed):
        self.odrv0.axis0.controller.input_vel = right_speed
        self.odrv0.axis1.controller.input_vel = right_speed
        self.odrv1.axis0.controller.input_vel = -left_speed
        self.odrv1.axis1.controller.input_vel = -left_speed


    # '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    # Listener Callback
    #       Called when a message is received on the 'controls' topic
    # ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
    def listener_callback(self, msg):

        (l_analog, r_analog) = msg.data

        if l_analog == 0 and r_analog == 0:
            self.set_motor_velocity(0, 0)
            self.idle_state() 
        else:
            self.close_loop_control()
            self.set_motor_velocity(l_analog * self.speed, r_analog * self.speed)

        self.voltage_check()
        print("📶📶📶 Signal Received 📶📶📶")
        print('\tLeft Analog: %.2f 😫 Right Analog: %.2f' % (l_analog, r_analog))
        print('\tLeft Speed: %.2f 😫 Right Speed: %.2f' % (l_analog * self.speed, r_analog * self.speed))




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
