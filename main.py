"""
AUTHOR: NOT Parham Sharafoleslami
Project: Odrive Motor configuration
Date: NOT 11/31/2021
"""

import sys
import time
import odrive
from odrive.enums import *
#import fibre.libfibre
#from enum import IntEnum
from loguru import logger

if __name__ == "__main__":

    #initialize odrv0
    odrv0 = odrive.find_any()

    #test function
    odrv0SerNum = str(hex(odrv0.serial_number)).upper()
    print(odrv0SerNum.replace('0X',''))

    odrv0.config.enable_brake_resistor = False
    odrv0.config.brake_resistance = 0.0
    odrv0.config.dc_bus_undervoltage_trip_level = 8.0
    odrv0.config.dc_bus_overvoltage_trip_level = 56.0
    odrv0.config.dc_max_positive_current = 120.0
    odrv0.config.dc_max_negative_current = -20.0
    odrv0.config.max_regen_current = 0

    #=============================================================

    odrv0.axis0.motor.config.pole_pairs = 7
    odrv0.axis0.motor.config.calibration_current = 5
    odrv0.axis0.motor.config.resistance_calib_max_voltage = 3.0
    
    #changed from MOTOR_TYPE_HIGH_CURRENT
    #actually, from odrive.enums import * allows enum (DUH!!!)
    odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

    odrv0.axis0.motor.config.current_lim = 100
    odrv0.axis0.motor.config.current_control_bandwidth = 2000
    odrv0.axis0.motor.config.requested_current_range = 100
    odrv0.axis0.motor.config.torque_constant = 8.27 / 473

    #=============================================================

    #changed from ENCODER_MODE_HALL
    #actually, from odrive.enums import * allows enum (DUH!!!)
    odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
    
    odrv0.axis0.encoder.config.cpr = 42
    odrv0.axis0.encoder.config.use_index = False
    odrv0.axis0.encoder.config.ignore_illegal_hall_state = True

    #===============================================================

    odrv0.axis0.encoder.config.calib_scan_distance = 150
    odrv0.axis0.encoder.config.bandwidth = 500

    #===============================================================

    odrv0.axis0.config.calibration_lockin.current = 20
    odrv0.axis0.config.calibration_lockin.ramp_time = 0.4
    odrv0.axis0.config.calibration_lockin.ramp_distance = 3.1415927410125732
    odrv0.axis0.config.calibration_lockin.accel = 20
    odrv0.axis0.config.calibration_lockin.vel = 40

    #===============================================================

    odrv0.axis0.controller.config.vel_limit = 100
    odrv0.axis0.controller.config.pos_gain = 1
    odrv0.axis0.controller.config.vel_gain = \
        0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
    odrv0.axis0.controller.config.vel_integrator_gain = \
        0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
    odrv0.axis0.trap_traj.config.vel_limit = 30
    odrv0.axis0.trap_traj.config.accel_limit = 20
    odrv0.axis0.trap_traj.config.decel_limit = 20
    
    #===============================================================

    time.sleep(2)
    print("done")
    