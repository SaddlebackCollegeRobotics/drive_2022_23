"""
AUTHOR: Maxwell Rehm & Matin Qurbanzadeh
Project: Odrive Motor configuration
Date: 10/17/2022
"""

import sys
import time
from tkinter import N, Y
import odrive
from odrive.enums import *
import fibre.libfibre
#from enum import IntEnum
from loguru import logger

if __name__ == "__main__":

    #finding odrive
    odrv0 = odrive.find_any()

    #test function -- getting serial number from the located odrive
    odrv0SerNum = str(hex(odrv0.serial_number)).upper()
    print(odrv0SerNum.replace('0X',''))

    # #==================================
    # #TODO: Want to fix the problem of it sometimes jumping to different odrives for calibration
    # #Testing two find_any() commands
    # #did not work, found the same one twice.
    # odrv1 = odrive.find_any()

    # #test function
    # odrv1SerNum = str(hex(odrv1.serial_number)).upper()
    # print(odrv1SerNum.replace('0X',''))
    #Next approach: look more into fibre.Domain for finer grained control over connection.
    #Can we use serial numbers to connect? We would need to have them preloaded into variables.
    #find_any has a parameter for serial numbers.


    #===================Reset=========================
    #If there were errors in the previous cycle, erase config would clear errors so you could start over
    logger.debug("Do you want to erase previous configuration? [Y/N]")
    ERcmd = input()

    if ERcmd.upper() == 'Y':
        try:
            odrv0.erase_configuration()

        except fibre.libfibre.ObjectLostError:
            pass

        odrv0 = odrive.find_any()
        print("Manual configuration erased.")
    #================================================

    
    #=============ODRIVE CONFIGURATION===============
    #Need to be set to true if we are using a psu with a brake resistor
    logger.debug("using power supply..? [Y/N]")
    PSUChoice = input()
    if PSUChoice.upper() == 'Y':
        odrv0.config.enable_brake_resistor = True
        #maybe create new if in future if using different resistor (ie not 2ohms)
        odrv0.config.brake_resistance = 2.0
    else:   
        odrv0.config.enable_brake_resistor = False
        odrv0.config.brake_resistance = 0.0
    #Odrivetool says the default value is 2.0 
    #(because the resitor that comes with the odrive is 50w 2ohm)
    #and to set it to default if not using br; look into this further.
    #If we are using a brake resistor change this value to resistor ohms.

    #
    odrv0.config.dc_bus_undervoltage_trip_level = 8.0
    odrv0.config.dc_bus_overvoltage_trip_level = 56.0
    odrv0.config.dc_max_positive_current = 120.0
    odrv0.config.dc_max_negative_current = -20.0
    odrv0.config.max_regen_current = 0
    #================================================

    #=============MOTOR CONFIGURATION================
    odrv0.axis0.motor.config.pole_pairs = 7
    odrv0.axis0.motor.config.resistance_calib_max_voltage = 3.0
    odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    odrv0.axis0.motor.config.requested_current_range = 100
    odrv0.axis0.motor.config.current_control_bandwidth = 2000
    odrv0.axis0.motor.config.current_lim = 100
    # 473 is Kv of our neo motor. (Kv = RPM at max throttle)
    odrv0.axis0.motor.config.torque_constant = 8.27 / 473
    #================================================

    #================ENCODER CONFIGURATION====================
    #Can use ENCODER_MODE_HALL as found in odrive.enums
    odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
    odrv0.axis0.encoder.config.cpr = 42
    #using an encoder with an index pin allows pre-calibration of the encoder and encoder index search
    #ours has index pins(Z) ; can set this to true
    odrv0.axis0.encoder.config.use_index = False
    #Changed from true to false got illegalhallstate big surprise.
    #When trying to request closed loop state and set vel = 3 got the following errors
    #MotorError.UNKNOWN_TORQUE and MotorError.UNKNOWN_VOLTAGE_COMMAND
    #Set this value to true and all 3 errors went away and it spun ; further research needed.
    odrv0.axis0.encoder.config.ignore_illegal_hall_state = True
    odrv0.axis0.encoder.config.calib_scan_distance = 150
    odrv0.axis0.encoder.config.bandwidth = 500
    #=========================================================


    odrv0.axis0.config.calibration_lockin.current = 20
    odrv0.axis0.config.calibration_lockin.ramp_time = 0.4
    odrv0.axis0.config.calibration_lockin.ramp_distance = 3.1415927410125732
    odrv0.axis0.config.calibration_lockin.accel = 20
    odrv0.axis0.config.calibration_lockin.vel = 40

    odrv0.axis0.controller.config.vel_limit = 100
    odrv0.axis0.controller.config.pos_gain = 1
    odrv0.axis0.controller.config.vel_gain = \
        0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
    odrv0.axis0.controller.config.vel_integrator_gain = \
        0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
    odrv0.axis0.trap_traj.config.vel_limit = 30
    odrv0.axis0.trap_traj.config.accel_limit = 20
    odrv0.axis0.trap_traj.config.decel_limit = 20

    odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH      #INPUT_MODE_VEL_RAMP
    odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    # saving the new configuration
    print("Saving manual configuration and rebooting...")
    try:
        odrv0.save_configuration()

    except fibre.libfibre.ObjectLostError:
        pass
    print("Manual configuration saved.")
    #After every save_configuration / erase_configuration / reboot we have to find odrive again.
    odrv0 = odrive.find_any()

    #===========================================================================================



    #===============================================================
    #INPUT_MODE_PASSTHROUGH
    odrv0.axis0.controller.config.input_mode = 1   #INPUT_MODE_VEL_RAMP

    #CONTROL_MODE_VELOCITY_CONTROL
    odrv0.axis0.controller.config.control_mode = 2

    #===============================================================


    input("Make sure the motor is free to move, then press enter...")
    logger.debug("Calibrating Odrive for NEO motor (you should hear a ""beep)...")

    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(15)
    
    if odrv0.axis0.motor.error != 0:
        print("fawked up at motor clibration QUIT NOW")
        print("hold ctrl")
        # To regenerate this file, nagivate to the top level of the ODrive repository and run:
        # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
        print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
        sys.exit()

    # logger.debug("full calibaration")
    # odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    # time.sleep(40)

    logger.debug("setting motor to precalibrated")
    odrv0.axis0.motor.config.pre_calibrated = True
    time.sleep(2)


    logger.debug("Calibrating Hall Polarity...")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
    time.sleep(15)

    if odrv0.axis0.encoder.error != 0:
        print("fawked up at Calibrating Hall Polarity QUIT NOW")
        print("hold ctrl")
        # To regenerate this file, nagivate to the top level of the ODrive repository and run:
        # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
        print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
        sys.exit()




    logger.debug("Calibrating Hall Phase...")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION
    time.sleep(15)

    if odrv0.axis0.encoder.error != 0:
        print("fawked up at Calibrating Hall Phase QUIT NOW")
        print("hold ctrl")
        # To regenerate this file, nagivate to the top level of the ODrive repository and run:
        # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
        print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
        sys.exit()

    




    logger.debug("Calibrating Hall Offset...")
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(25)

    if odrv0.axis0.encoder.error != 0:
        print("fawked up at Calibrating Hall Offset QUIT NOW")
        print("hold ctrl")
        # To regenerate this file, nagivate to the top level of the ODrive repository and run:
        # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
        print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
        sys.exit()

    

    

    logger.debug("setting encoder to precalibrated...")
    odrv0.axis0.encoder.config.pre_calibrated = True
    time.sleep(2)

    logger.debug("trying to save...")
    odrv0.save_configuration()
    logger.debug("saved...")
    


    print("it all worked, run motor? [Y / N]")
    runMotorChoice = input()
    if runMotorChoice.upper() == 'Y':
        
        print("do you want set velocity? [Y / N]")
        setVelocityYN = input()
        
        if setVelocityYN.upper() == 'Y':
            print("enter int for velocity. 1-10 is fine,\n anything from 10-70 hold the motor still")
            odrv0.axis0.controller.input_vel = input()
            print(odrv0.axis0.controller.input_vel)
        
        print("set to closed loop control? (motor will spin). [Y / N]")
        setCLC_YN = input()
        if setCLC_YN.upper() == 'Y':
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            input("press enter to stop...")
            odrv0.axis0.requested_state = AXIS_STATE_IDLE

    

    #YES
