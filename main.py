"""
AUTHOR: Max Rehm & Matin Qurbanzadeh
Project: Odrive Motor configuration
Date: 10/19/2022
"""

import sys
import time
import odrive
from odrive.enums import *
import fibre.libfibre
#from enum import IntEnum
from loguru import logger



#import contexlib
 #       with contextlib.suppress(fibre.libfibre.ObjectLostError):
  #          odrv_num.erase_configuration()

if __name__ == "__main__":

    #finding odrive
    odrives = []


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



    def config_motor(odrv_num, axis_num, shouldClear):

        #===================Reset=========================
        #If there were errors in the previous cycle, erase config would clear errors so you could start over
        if shouldClear:
            try:
                odrv_num.erase_configuration()

            except fibre.libfibre.ObjectLostError:
                pass


        #FIX THIS.
        odrv_num = odrive.find_any()
        print("Manual configuration erased.")
        #================================================

        axis = getattr(odrv_num, f'axis{axis_num}')
        
        #test function -- getting serial number from the located odrive
        odrvSerNum = str(hex(odrv_num.serial_number)).upper()
        print("Serial Number of connected odrive: ",odrvSerNum.replace('0X',''))

        #=============ODRIVE CONFIGURATION===============
        #Need to be set to true if we are using a psu with a brake resistor
        logger.debug("using power supply..? [Y/N]")
        PSUChoice = input()
        if PSUChoice.upper() == 'Y':
            odrv_num.config.enable_brake_resistor = True
            #maybe create new if in future if using different resistor (ie not 2ohms)
            odrv_num.config.brake_resistance = 2.0
        else:   
            odrv_num.config.enable_brake_resistor = False
            odrv_num.config.brake_resistance = 0.0
        #Odrivetool says the default value is 2.0 
        #(because the resitor that comes with the odrive is 50w 2ohm)
        #and to set it to default if not using br; look into this further.
        #If we are using a brake resistor change this value to resistor ohms.


        odrv_num.config.dc_bus_undervoltage_trip_level = 8.0
        odrv_num.config.dc_bus_overvoltage_trip_level = 56.0
        odrv_num.config.dc_max_positive_current = 120.0
        odrv_num.config.dc_max_negative_current = -20.0
        odrv_num.config.max_regen_current = 0
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
            odrv_num.save_configuration()

        except fibre.libfibre.ObjectLostError:
            pass
        print("Manual configuration saved.")
        


    def calib_motor(odrv_num, axis_num, testMotor):
        #=======================================CALIBRATION SEQUENCE==============================================

        axis = getattr(odrv_num, f'axis{axis_num}')

        #===============================================================
        #INPUT_MODE_PASSTHROUGH
        axis.controller.config.input_mode = 1   #INPUT_MODE_VEL_RAMP

        #CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.control_mode = 2

        #===============================================================

        #THESE CALIBRATION STATES HAVE TO BE IN ORDER, ELSE IT WILL BE MEAN.
        input("Make sure the motor is free to move, then press enter...")
        logger.debug("Calibrating Odrive for NEO motor (you should hear a beep)...")


        #==============================MOTOR CALIBRATION=================================

        # MEASURING PHASE RESISTANCE/INDUCTANCE OF MOTOR
        # to store these values, do motor.config.pre_calibrated = True, as we do below.
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        # Sleep to allow the motor to finish the calibrate process.
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
            print("t " ,axis.current_state)
        
        
        # If there was an error during motor calibration, exit and link to error list.
        if axis.motor.error != 0:
            print("Error at motor clibration QUIT NOW")
            print("hold ctrl  v")
            # To regenerate this file, nagivate to the top level of the ODrive repository and run:
            # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
            print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
            sys.exit()

        #================================================================================

        #================================ENCODER CALIBRATION===============================
        # This stores motor.config.phase_resistance and motor.config.phase_inductance to the odrive memory.
        logger.debug("Setting motor to precalibrated")
        axis.motor.config.pre_calibrated = True
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
            print(axis.current_state)
        time.sleep(5)
        print("t " ,axis.current_state)

        # Rotate the motor in lockin and calibrate hall polarity
        logger.debug("Calibrating Hall Polarity...")
        axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
            print(axis.current_state)
        print(axis.current_state)
        time.sleep(5)
        print("t " ,axis.current_state)

        # If there was an error during encoder polarity calibration, exit and link to error list.
        if axis.encoder.error != 0:
            print("Error at Calibrating Hall Polarity QUIT NOW")
            print(axis.encoder.error)
            print("hold ctrl  v")
            # To regenerate this file, nagivate to the top level of the ODrive repository and run:
            # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
            print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
            sys.exit()


        # Rotate the motor for 30s to calibrate hall sensor edge offsets
        # Note: The phase offset is not calibrated at this time, so the map is only relative
        logger.debug("Calibrating Hall Phase...")
        axis.requested_state = AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
            print(axis.current_state)
        print(axis.current_state)
        time.sleep(5)
        print("t " ,axis.current_state)

        # If there was an error during encoder phase calibration, exit and link to error list.
        if axis.encoder.error != 0:
            print("Error at Calibrating Hall Phase QUIT NOW")
            print("hold ctrl")
            # To regenerate this file, nagivate to the top level of the ODrive repository and run:
            # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
            print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
            sys.exit()

        


        # Turns the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase.
        # Needs motor to be calibrated
        # If successful, encoder calibration will make the encoder.is_ready == True
        logger.debug("Calibrating Hall Offset...")
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
            print(axis.current_state)

        print(axis.current_state)
        time.sleep(5)
        print("t " ,axis.current_state)

        # If there was an error during encoder offset calibration, exit and link to error list.
        if axis.encoder.error != 0:
            print("Error at Calibrating Hall Offset QUIT NOW")
            print("hold ctrl")
            # To regenerate this file, nagivate to the top level of the ODrive repository and run:
            # python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py
            print("https://github.com/odriverobotics/ODrive/blob/master/tools/odrive/enums.py")
            sys.exit()



        logger.debug("Setting encoder to precalibrated...")
        axis.encoder.config.pre_calibrated = True
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
            print(axis.current_state)

        print(axis.current_state)
        time.sleep(5)
        print("t " ,axis.current_state)

        logger.debug("trying to save...")
        # saving the new configuration
        print("Saving manual configuration and rebooting...")
        try:
            odrv_num.save_configuration()

        except fibre.libfibre.ObjectLostError:
            pass
        print("Manual configuration saved.")
        logger.debug("saved...")

        odrv_num = odrive.find_any()
        axis = getattr(odrv_num, f'axis{axis_num}')

        #==================================================================================

    
        #===================================RUN SEQUENCE===================================
        if testMotor == 'Y':
            print("[Y / N] it all worked, run motor?")
            runMotorChoice = input()
            if runMotorChoice.upper() == 'Y':
                
                print("enter value for velocity. 1-10 is fine,\n anything from 10-70. Hold the motor still. Enter X to quit..")
                velVal = input()

                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

                while True:
                    try:
                        axis.controller.input_vel = int(velVal)
                        print("Velocity set to: ",axis.controller.input_vel)
                        print("Enter a value (hit enter to exit): ")
                        velVal = input()

                    except ValueError:
                        axis.requested_state = AXIS_STATE_IDLE
                        break
                    
                

        #==================================================================================

    
    logger.debug("Would you like to test each motor individually after each is calibrated[Y], if not just hit enter...")
    logger.debug("Note: You will still be prompted to test all motors at once.")
    motorTestCMD = input().upper()

    
    #loop through each odrive that is connected and configure/calibrate each axis_num.
    #for odrv in odrives:

    config_motor(odrives[0], 0, True)
    #After every save_configuration / erase_configuration / reboot we have to find odrive again.
    odrives[0] = odrive.find_any(serial_number='207937815753')
    calib_motor(odrives[0], 0, motorTestCMD)
    config_motor(odrives[0], 1, False)
    odrives[0] = odrive.find_any()
    calib_motor(odrives[0], 1, motorTestCMD)
    #YES 


#TODO: 10/20/2022 USE MAX'S FILE TO RUN FUNCTIONS 