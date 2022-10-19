"""
AUTHOR: NOT Parham Sharafoleslami
Project: Odrive Motor configuration
Date: NOT 11/31/2021
"""

import sys
import time
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
    logger.debug("Do you want to erase? [Y/N]")
    ERcmd = input()

    if ERcmd.upper() == 'Y':
        try:
            odrv0.erase_configuration()

        except fibre.libfibre.ObjectLostError:
            pass

        odrv0 = odrive.find_any()

    logger.debug("Manual configuration erased.")
    #================================================

    
    #=============ODRIVE CONFIGURATION===============
    odrv0.config.enable_brake_resistor = False
    odrv0.config.brake_resistance = 0.0
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
    odrv0.axis0.motor.config.torque_constant = 8.27 / 473
    #================================================

    #================ENCODER CONFIGURATION====================
    #Can use ENCODER_MODE_HALL as found in odrive.enums
    odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
    odrv0.axis0.encoder.config.cpr = 42
    #using an encoder with an index pin to allow pre-calibration of the encoder and encoder index search
    #ours has index pins ; set it to true
    odrv0.axis0.encoder.config.use_index = False
    #Changed from true to false
    odrv0.axis0.encoder.config.ignore_illegal_hall_state = False
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
    logger.debug("Saving manual configuration and rebooting...")
    try:
        odrv0.save_configuration()

    except fibre.libfibre.ObjectLostError:
        pass
    logger.debug("Manual configuration saved.")
    #After every save_configuration / erase_configuration / reboot we have to find odrive again.
    odrv0 = odrive.find_any()


