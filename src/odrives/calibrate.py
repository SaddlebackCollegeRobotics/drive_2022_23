"""
AUTHOR: Max Rehm & Matin Qurbanzadeh
Project: Odrive Motor configuration
Date: 10/19/2022
"""

import sys
import time
import odrive
from odrive.enums import *

from loguru import logger
import threading
import time

from fibre.libfibre import ObjectLostError
from find_serial import get_all_odrives
from configure import config_motor



def precalibration(odrv_num, axis, axis_num):
    axis.motor.config.pre_calibrated = True
    print("\nOdriveSN: ",odrv_num, "\nState: ", axis.current_state, "\nAxis: ", axis_num)
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(2)

    print("\nOdriveSN: ",odrv_num, "\nState: ", axis.current_state, "\nAxis: ", axis_num)
    time.sleep(5)



def calib_phase(odrv_num, axis, axis_num, ENUM_PHASE, calib_string):
    axis.requested_state = ENUM_PHASE
    print("OdriveSN: ",odrv_num, "\nState: ", axis.current_state, "\nAxis: ", axis_num)
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(2)
    
    print("OdriveSN: ",odrv_num, "\nState: ", axis.current_state, "\nAxis: ", axis_num)
    
    if axis.motor.error != 0:
        logger.error("Error at ", calib_string, " 😢")
        print("Odrive: ", odrv_num, "Axis: ", axis_num)
        print(axis.motor.error)
        print("hold ctrl  v")
        print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
        sys.exit()



def calib_motor(odrv_num, axis_num):
    #=======================================CALIBRATION SEQUENCE==============================================

    odrv = odrive.find_any(serial_number=odrv_num)
    axis = getattr(odrv, f'axis{axis_num}')

    #===============================================================
    #INPUT_MODE_PASSTHROUGH
    axis.controller.config.input_mode = 1   #INPUT_MODE_VEL_RAMP

    #CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.control_mode = 2



    #==============================MOTOR CALIBRATION=================================
    calib_phase(odrv_num, axis, axis_num, AXIS_STATE_MOTOR_CALIBRATION, "Motor Calibration")



    #================================ENCODER CALIBRATION===============================
    # This stores motor.config.phase_resistance and motor.config.phase_inductance to the odrive memory.
    logger.debug("Setting motor to precalibrated... 😎️")
    precalibration(odrv_num, axis, axis_num)
    


    # Rotate the motor in lockin and calibrate hall polarity
    logger.debug("Calibrating Hall Polarity... 🤞")
    calib_phase(odrv_num, axis, axis_num, AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION, "Calibrating Hall Polarity")



    # Rotate the motor for 30s to calibrate hall sensor edge offsets
    # Note: The phase offset is not calibrated at this time, so the map is only relative
    logger.debug("Calibrating Hall Phase... 🤞")
    calib_phase(odrv_num, axis, axis_num, AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION, "Calibrating Hall Phase")
    


    # Turns the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase.
    # Needs motor to be calibrated
    # If successful, encoder calibration will make the encoder.is_ready == True
    logger.debug("Calibrating Hall Offset... 🤞")
    calib_phase(odrv_num, axis, axis_num, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, "Calibrating Hall Offset")



    logger.debug("Setting encoder to precalibrated... 😎️")
    precalibration(odrv_num, axis, axis_num)



    #======================================SAVE============================================
    logger.debug("trying to save...")
    # saving the new configuration
    print("Saving manual configuration and rebooting... 😎️")
    try:
        odrv.save_configuration()

    except ObjectLostError:
        pass
    print("Manual configuration saved.")
    logger.debug("saved...")

    odrv = odrive.find_any(serial_number=odrv_num)
    axis = getattr(odrv, f'axis{axis_num}')



def calibrate_all_motors(odrv0, odrv1):
    print("Odrive 0: ", odrv0, "Odrive 1: ", odrv1)
    
    config_motor(odrv0, 0, True, False)
    config_motor(odrv0, 1, False, False)
    config_motor(odrv1, 0, False, False)
    config_motor(odrv1, 1, False, False)
    


    t00 = threading.Thread(target=calib_motor, args=(odrv0, 0)) 
    print("created thread 00")
    t01 = threading.Thread(target=calib_motor, args=(odrv0, 1)) 
    print("created thread 01")

    t10 = threading.Thread(target=calib_motor, args=(odrv1, 0)) 
    print("created thread 10")
    t11 = threading.Thread(target=calib_motor, args=(odrv1, 1)) 
    print("created thread 11")



    t00.start()
    print("thread 00 started")
    time.sleep(1)
    t01.start()
    print("thread 01 started")
    time.sleep(1)

    t10.start()
    print("thread 10 started")
    time.sleep(1)
    t11.start()
    print("thread 11 started")
    time.sleep(1)


    t00.join()
    t01.join()

    t10.join()
    t11.join()

    print("all threads done 😎️ 😎️ 😎️ 😎️")