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
import subprocess

from fibre.libfibre import ObjectLostError



def config_motor(odrv_num, axis_num, shouldClear, PSUChoice):

    #===================Reset=========================
    #If there were errors in the previous cycle, erase config would clear errors so you could start over
    odrv = odrive.find_any(serial_number=odrv_num)

    if shouldClear:
        try:
            odrv.erase_configuration()
        except ObjectLostError:
            pass

        odrv = odrive.find_any(serial_number=odrv_num)
        
        print("Erased Previous Configuration... 🗑️")
    #================================================

    axis = getattr(odrv, f'axis{axis_num}')
    
    #test function -- getting vbus_voltage to prove this is a unique odrive
    vbus_voltage = odrv.vbus_voltage
    print("Serial Number of connected odrive: ", odrv_num)
    print("VBUS_voltage of connected odrive: ", vbus_voltage)

    #=============ODRIVE CONFIGURATION===============
    #Need to be set to true if we are using a psu with a brake resistor
    
    if PSUChoice:
        odrv.config.enable_brake_resistor = True
        #maybe create new if in future if using different resistor (eg not 2ohms)
        odrv.config.brake_resistance = 2.0
    else:   
        odrv.config.enable_brake_resistor = False
        odrv.config.brake_resistance = 0.0
    #Odrivetool says the default value is 2.0 
    #(because the resitor that comes with the odrive is 50w 2ohm)
    #and to set it to default if not using br; look into this further.
    #If we are using a brake resistor change this value to resistor ohms.


    odrv.config.dc_bus_undervoltage_trip_level = 8.0
    odrv.config.dc_bus_overvoltage_trip_level = 56.0
    odrv.config.dc_max_positive_current = 120.0
    odrv.config.dc_max_negative_current = -20.0
    odrv.config.max_regen_current = 0
    #================================================

    #=============MOTOR CONFIGURATION================
    axis.motor.config.pole_pairs = 7
    axis.motor.config.resistance_calib_max_voltage = 3.0
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    axis.motor.config.requested_current_range = 100
    axis.motor.config.current_control_bandwidth = 2000
    axis.motor.config.current_lim = 100
    # 473 is Kv of our neo motor. (Kv = RPM at max throttle)
    axis.motor.config.torque_constant = 8.27 / 473
    #================================================

    #================ENCODER CONFIGURATION====================
    #Can use ENCODER_MODE_HALL as found in odrive.enums
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 42
    #using an encoder with an index pin allows pre-calibration of the encoder and encoder index search
    #ours has index pins(Z) ; can set this to true
    axis.encoder.config.use_index = False
    #changed this to false, wasnt here before. default was true.
    axis.encoder.config.use_index_offset = False
    #Changed from true to false got illegalhallstate big surprise.
    #When trying to request closed loop state and set vel = 3 got the following errors
    #MotorError.UNKNOWN_TORQUE and MotorError.UNKNOWN_VOLTAGE_COMMAND
    #Set this value to true and all 3 errors went away and it spun ; further research needed.
    axis.encoder.config.ignore_illegal_hall_state = True
    axis.encoder.config.calib_scan_distance = 150
    axis.encoder.config.bandwidth = 500
    #=========================================================


    axis.config.calibration_lockin.current = 20
    axis.config.calibration_lockin.ramp_time = 0.4
    axis.config.calibration_lockin.ramp_distance = 3.1415927410125732
    axis.config.calibration_lockin.accel = 20
    axis.config.calibration_lockin.vel = 40

    axis.controller.config.vel_limit = 100
    axis.controller.config.pos_gain = 1
    axis.controller.config.vel_gain = \
        0.02 * axis.motor.config.torque_constant * axis.encoder.config.cpr
    axis.controller.config.vel_integrator_gain = \
        0.1 * axis.motor.config.torque_constant * axis.encoder.config.cpr
    axis.trap_traj.config.vel_limit = 30
    axis.trap_traj.config.accel_limit = 20
    axis.trap_traj.config.decel_limit = 20

    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH      #INPUT_MODE_VEL_RAMP
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

    # saving the new configuration
    print("Saving manual configuration and rebooting...")
    try:
        odrv.save_configuration()
    except ObjectLostError:
        pass
    print("Manual configuration saved.")



# def get_all_odrives():

#     odrivesSerNum = []

#     usbDevices = str(subprocess.run(['lsusb', '-v'], capture_output=True).stdout).split('\\n')

#     for device in usbDevices:
#         if "Serial" in device:
#             odrivesSerNum.append(device[28:].strip())
#             odrivesSerNum = list(filter(None, odrivesSerNum))

#     return odrivesSerNum
def get_all_odrives():

    odrivesSerialList = []

    # This will pull all devices connected to the computer
    usbDevices = str(subprocess.run(['lsusb', '-v'], capture_output=True).stdout).split('\\n')

    odriveFound = False

    # This will iterate line by line through the usbDevices[]
    for line in usbDevices:

        if "ODrive" in line:
            odriveFound = True
            print(line)

        # This will pull any device with a serial number 
        if odriveFound and "Serial" in line:
            odrivesSerialList.append(line[28:].strip())
            #odrivesSerialList = list(filter(None, odrivesSerialList))
            odriveFound = False

    return odrivesSerialList



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
    print("Odrive 0: ", odrv0, "\nOdrive 1: ", odrv1, "\n\n\n")
    
    config_motor(odrv0, 0, True, False)
    config_motor(odrv0, 1, False, False)
    config_motor(odrv1, 0, True, False)
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
    print("\n\n\nBEGINNING DRIVE CONTROL\n\n\n")