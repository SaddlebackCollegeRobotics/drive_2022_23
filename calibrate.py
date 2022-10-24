import sys
import time
import odrive
from odrive.enums import *
from loguru import logger
from fibre.libfibre import ObjectLostError

def calib_motor(odrv_num, axis_num, testMotor):
        #=======================================CALIBRATION SEQUENCE==============================================

        odrv = odrive.find_any(serial_number=odrv_num)

        axis = getattr(odrv, f'axis{axis_num}')

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
            odrv.save_configuration()

        except ObjectLostError:
            pass
        print("Manual configuration saved.")
        logger.debug("saved...")

        odrv = odrive.find_any(serial_number=odrv_num)
        axis = getattr(odrv, f'axis{axis_num}')

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

    