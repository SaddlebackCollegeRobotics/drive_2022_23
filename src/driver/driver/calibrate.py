# ========================================================================================
# Author:   Max Rehm (@max9001)    &    Matin Qurbanzadeh (@MatinQurban)
# Modified: Jasper Doan (@wluxie)
# Date:     10/19/2022
# ========================================================================================
# Description:
#   This is the Odrive calibration API. It is used to calibrate the Odrive motors.
#   It is used in the ROS2 node 'drive_receiver_sub.py', and Autonomy's startup script.
#
# ========================================================================================
# Odrive NEO Brushless Motor Configuration:
#   For more information on how we got these values, please refer to the Odrive documentation
#       https://docs.odriverobotics.com/
#   And check out our notes on the Odrive configuration here:
#       https://docs.google.com/spreadsheets/d/1GjdOlduLwA-kQ82NoD1YQMw4ON5OlDftQ1XObfu-lJc/edit?usp=sharing
#
# ========================================================================================
# Dependencies:
#   - odrive
#   - loguru
#   - fibre
# ========================================================================================

import sys
import time
import odrive
from odrive.enums import *

from loguru import logger
import threading
import time
import subprocess

from fibre.libfibre import ObjectLostError




def erase_config(odrv_num, clear):
    if clear:   # I'd suggest not touching this if and try catch throw
        try:
            odrive.find_any(serial_number=odrv_num).erase_configuration()
            print("\tErased Previous Configuration... 🗑️")
        except:
            print("\tFailed to Erased Previous Configuration... 🗑️")
            pass
    return odrive.find_any(serial_number=odrv_num)




def save_config(odrv_num):
    odrv = odrive.find_any(serial_number=odrv_num)
    print("Saving manual configuration and rebooting...")
    try:
        odrv.save_configuration()
        print("\tSaved Configuration... 📝")
    except ObjectLostError:
        print("\tFailed to Save Configuration... 📝")
        pass




def config_motor(odrv_num, axis_num, clear, powerDC):
    odrv = odrive.find_any(serial_number=odrv_num)
    axis = getattr(odrv, f'axis{axis_num}')
    vbus_voltage = odrv.vbus_voltage



    # ==== CLEAR PREVIOUS CONFIGURATION ================================
    # =================================================================
    odrv = erase_config(odrv_num, clear)
    print(">>> ODrive's SN:\t", odrv_num, "<<<")
    print(">>> ODrive's Voltage:\t", vbus_voltage, "<<<")



    # ==== ODRIVE CONFIGURATION ========================================
    # ==================================================================

    # ------------------------------------------------------------------
    #   Enable Brake Resistor:  Enable the brake resistor if power supply is DC
    #   Brake Resistor:         2 ohms because the resitor that comes with the odrive is 50w 2ohm
    odrv.config.enable_brake_resistor = powerDC                 # Enable brake resistor if power supply is DC
    odrv.config.brake_resistance = 2.0 if powerDC else 0.0      # Set brake resistor to 2 ohms if power supply is DC


    # ------------------------------------------------------------------
    #   DC Bus Voltage Limits:  Set the DC bus voltage limits
    #   DC Bus Current Limits:  Set the DC bus current limits
    #   Max Regen Current:      Set the maximum regen current
    odrv.config.dc_bus_undervoltage_trip_level = 8.0
    odrv.config.dc_bus_overvoltage_trip_level = 56.0
    odrv.config.dc_max_positive_current = 120.0
    odrv.config.dc_max_negative_current = -20.0
    odrv.config.max_regen_current = 0


    # ------------------------------------------------------------------
    #   Motor Configuration:
    #       Pole Pairs:          Number of pole pairs in the motor
    #       Resistance Calib:    Maximum voltage to apply during resistance calibration
    #       Motor Type:          Type of motor
    #       Current Range:       Maximum current that the motor can draw
    #       Current Control:     Current control bandwidth
    #       Current Limit:       Maximum current that the motor can draw
    #       Torque Constant:     Torque constant of the motor
    axis.motor.config.pole_pairs = 7                            # Brushless NEO motors have 7 pole pairs
    axis.motor.config.resistance_calib_max_voltage = 3.0        # Maximum voltage to apply during resistance calibration
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT      # Brushless NEO motors are high current motors
    axis.motor.config.requested_current_range = 100             # Maximum current that the motor can draw
    axis.motor.config.current_control_bandwidth = 2000          # Should be at least 10x the motor's current range
    axis.motor.config.current_lim = 100                         # Maximum current that the motor can draw
    axis.motor.config.torque_constant = 8.27 / 473              # Brushless NEO motors have a torque constant of 8.27 mNm/Amp 
                                                                #   and 473 is Kv of our neo motor. (Kv = RPM at max throttle)

    # ------------------------------------------------------------------
    #   Encoder Configuration:
    #       Mode:               Encoder mode
    #       CPR:                Counts per revolution
    #       Use Index:          Use index pulse
    #       Use Index Offset:   Use index pulse offset
    #       Ignore Illegal:     Ignore illegal hall state errors
    #       Calib Scan:         Calibration space scan distance (in encoder counts)
    #       Bandwidth:          Encoder bandwidth
    axis.encoder.config.mode = ENCODER_MODE_HALL                # Using an encoder with an index pin allows pre-calibration of
    axis.encoder.config.cpr = 42                                #   the encoder and encoder index search. NEO Brushless has index 
    axis.encoder.config.use_index = False                       #   pins(Z) --> Can set this to true
    axis.encoder.config.use_index_offset = False
    axis.encoder.config.ignore_illegal_hall_state = True        # Leave this as true, or else the motor will yell at you
    axis.encoder.config.calib_scan_distance = 150
    axis.encoder.config.bandwidth = 500


    # ------------------------------------------------------------------
    #   Controller Configuration & Axis Calibration:
    #       Axis Calib Current: Current to use during calibration
    #       Axis Calib Ramp:    Ramp distance to use during calibration
    #       Axis Calib Accel:   Acceleration to use during calibration
    #       Axis Calib Vel:     Velocity to use during calibration
    axis.config.calibration_lockin.current = 20
    axis.config.calibration_lockin.ramp_time = 0.4
    axis.config.calibration_lockin.ramp_distance = 3.1415927410125732
    axis.config.calibration_lockin.accel = 20
    axis.config.calibration_lockin.vel = 40

    #       Vel Limit:          Velocity limit for controller
    #       Pos Gain:           Position gain from controller
    #       Vel Gain:           Velocity gain from controller
    #       Integrator Gain:    Velocity integrator gain from controller
    #       Vel Limit:          Velocity limit for trajectory planner
    #       Accel Limit:        Acceleration limit for trajectory planner 
    #       Decel Limit:        Deceleration limit for trajectory planner   
    axis.controller.config.vel_limit = 100
    axis.controller.config.pos_gain = 1
    axis.controller.config.vel_gain = \
        0.02 * axis.motor.config.torque_constant * axis.encoder.config.cpr
    axis.controller.config.vel_integrator_gain = \
        0.1 * axis.motor.config.torque_constant * axis.encoder.config.cpr
    axis.trap_traj.config.vel_limit = 30
    axis.trap_traj.config.accel_limit = 20
    axis.trap_traj.config.decel_limit = 20

    #       Input Mode:         Input mode for controller
    #       Control Mode:       Control mode for controller
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL    



    # ==== SAVE CONFIGURATION ==========================================
    # ==================================================================
    save_config(odrv_num)




def get_all_odrives():
    # Pull all connected devices ID to the list
    odrivesSerialList = []
    usbDevices = str(subprocess.run(['lsusb', '-v'], capture_output=True).stdout).split('\\n')

    odriveFound = False
    # Find all ODrives connected to the computer
    for line in usbDevices:
        if "ODrive" in line:
            odriveFound = True
            print(line)

        if odriveFound and "Serial" in line:
            odrivesSerialList.append(line[28:].strip())
            odriveFound = False

    return odrivesSerialList



def calib_motor(odrv_num, axis_num):
    odrv = odrive.find_any(serial_number=odrv_num)
    axis = getattr(odrv, f'axis{axis_num}')

    axis.controller.config.input_mode = 1   # INPUT_MODE_PASSTHROUGH & INPUT_MODE_VEL_RAMP
    axis.controller.config.control_mode = 2 # CONTROL_MODE_VELOCITY_CONTROL

    # ---- Target List ------------------------------------------------
    calib_targets = {
        "Motor Calibration" : [AXIS_STATE_MOTOR_CALIBRATION, axis.motor.error],
        "Hall Polarity"     : [AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION, axis.encoder.error],
        "Hall Phase"        : [AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION, axis.encoder.error],
        "Hall Offset"       : [AXIS_STATE_ENCODER_OFFSET_CALIBRATION, axis.encoder.error]
    }

    axis_target = {
        "motor" : axis.motor,
        "encoder" : axis.encoder
    }

    # ---- Calibration Function ---------------------------------------
    def calib(target : str):
        logger.debug("Calibrating {}... 🤞".format(target))
        axis.requested_state = calib_targets[target][0]
        debug_idle_log()
        if calib_targets[target][1] != 0:
            logger.error("Error at {} 😢".format(target))
            print("\t> Error: ", calib_targets[target][1])
            sys.exit()

    # ---- Pre-Calibration Function -----------------------------------
    def pre_calib(target : str):
        logger.debug("Setting {} to precalibrated... 😎️".format(target))
        axis_target[target].config.pre_calibrated = True
        debug_idle_log()

    # ---- Debugging Function -----------------------------------------
    def debug_idle_log():
        print("\t> OdriveSN: ",odrv_num, " -- Axis: ", axis_num, " -- State: ", axis.current_state, " <")
        while axis.current_state != AXIS_STATE_IDLE:
            time.sleep(2)
        print("\t> OdriveSN: ",odrv_num, " -- Axis: ", axis_num, " -- State: ", axis.current_state, " <")
        time.sleep(5)
        

    # ==================================================================
    # NOTE:
    #   The following code is for calibrating the motor and it NEEDS TO
    #   BE IN THIS ORDER. If you change the order, the motor will yell at
    #   you and you will cry.
    # ==================================================================\
    calib("Motor Calibration")
    pre_calib("motor")
    calib("Hall Polarity")
    calib("Hall Phase")
    calib("Hall Offset")
    pre_calib("encoder")


    save_config(odrv_num)
    odrv = odrive.find_any(serial_number=odrv_num)
    axis = getattr(odrv, f'axis{axis_num}')



def calibrate_all_motors(odrv0, odrv1):
    print("Odrive 0: ", odrv0, "\nOdrive 1: ", odrv1, "\n\n\n")
    
    config_motor(odrv0, 0, True, False)
    config_motor(odrv0, 1, False, False)
    config_motor(odrv1, 0, True, False)
    config_motor(odrv1, 1, False, False)
    

    t00 = threading.Thread(target=calib_motor, args=(odrv0, 0)) 
    print("> Odrv0 [ M0 ] - Created thread O0_M0")
    t01 = threading.Thread(target=calib_motor, args=(odrv0, 1)) 
    print("> Odrv0 [ M1 ] - Created thread O0_M1")

    t10 = threading.Thread(target=calib_motor, args=(odrv1, 0)) 
    print("> Odrv1 [ M0 ] - Created thread O1_M0")
    t11 = threading.Thread(target=calib_motor, args=(odrv1, 1)) 
    print("> Odrv1 [ M1 ] - Created thread O1_M1")


    t00.start()
    print("-- Thread O0_M0 started --")
    time.sleep(1)
    t01.start()
    print("-- Thread O0_M1 started --")
    time.sleep(1)

    t10.start()
    print("-- Thread O1_M0 started --")
    time.sleep(1)
    t11.start()
    print("-- Thread O1_M1 started --")
    time.sleep(1)


    t00.join()
    t01.join()

    t10.join()
    t11.join()

    print("▷ All threads done 😎️ 😎️ 😎️ 😎️")
    print("\n\n\n>>> BEGINNING DRIVE CONTROL <<<\n\n\n")



# ████████████████████████████████████████████████████████████████████████████████
# ██████████████████████████████████████████▀▀▀███▀█▀█████████████████████████████
# ██████████████████████████████████▀▀``               '╙▀████████████████████████
# ████████████████████████████▀▀              ╓  , ╦╗      `██████████████████████
# █████████████████████▀▀"`            ╓ ▄▄,, , ▄▓▄╫φ▀▄.µ╔,Φ▄█████████████████████
# █████████████████▀'         ,;-,▄▄"╢▀▀██▓███▌████▓█ ╙████▌╬ ¥▀██████████████████
# ███████████████╨      . ¥ ▓▓▌▀╚ ╚▀"╨▌ ║▓▀████▌`█╙▌▀   ╛`"`ΦH  `█████████████████
# █████████████"           N ╙╫┐¼    φ║▄╢█▌j╙╣█▌                  ████████████████
# ███████████▀              ▓ ,▌ W   ╙▄▀▄▀▓▄¥  "                   ███████████████
# ██████████"                ▀'.Y¥╫╥  `W╙▀▀Å¥`                     ╢██████████████
# █████████▌                  ╙Kφ╙Ü▀█▄  `≈`     \▄  ╙ µ            '██████████████
# █████████▌          ▄▓▓▓H     `╝▀▓M"▀╗µ   ,,    ╙    "º-          ║█████████████
# █████████H        ▄██████▄≈╔,w--╦▓▀▄▄ %2*   ``           «« ⁿ≈,;╓╓⌂▀████████████
# █████████  :     ╝█████████▓▓▓▓▓▀██▓█▓▌╣m≥,   `        ╗╓,,╟µ▓█████╕████████████
# ████████▌       ╔▀▀██████▓██████████████████ÑÜ« ,;╦▄▄▓██████████████Ö███████████
# ████████▌    ▓ ",╬██▀▓████▀▀▀▀▐▄▄▄██████████████████████████████████████████████
# ████████▌    ╙φM`╜╣▓█████▄µÇ╓▄███████████████████████████████████████▐██████████
# ████████M        ╣███████████████████████████████████████████████████ ██████████
# ████████         ╠▓██████████████████████████████████████████████████⌐╫█████████
# ████████        ╔╚▓██████████████████████████████████████████████████M╫█████████
# ████████        ╚╨║██████████████████████████████████████████████████M╫█████████
# ████████         ╔▓██████████████████████████████████████████████████▌╫█████████
# ███████H        ╫████████████████████████████████████████████████████▌▀█████████
# ███████H      j▄╫██████████████████▓▌▀▀▀▀████████████████████████████▌  ""▀▀████
# ███████H      "▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀ª¥══╙▀▀█████████▀▀╙╙``                 ██
# ███▀▀███  ╛                                     ``                            ██
# ██▌H  `▀▌   »Φ▓▄▄,                                                           ███
# ███▌ ,  `   ╔█████Ω µ╥╗╗µ,╓╥╓««     '%                                       ███
# ██╝████▌ '  ╢█▀████ ╟╫╫╬▓▀╙`            ```    ,▄▓█▓     `                   ███
# ██▌████▄  N    ╙███╕```                       ▓██████µ                      ▄███
# ███▓█████▓MH    ▐█▀                          ▓████████w                   ╓█████
# ███▓▌████▓▄`    "  ]U                       ███████████▄                ▄███████
# ████▓▄████▌╫w      ]╫╬▄▄5*╤≈▄╥,,,,,╓«« ,╓═╓█████████████▄\««╥╥,╥..      ████████
# █████▓█████╨█   .,»`╫▓▓▓██▓▓▓▓ΦΦΦ╬╫╫╫╫╫╫▄▓███████████████▄⌂`""╙"""`     ████████
# ███████▓███ ║█▄╓██▌µ╦╫╫▓▓██████████████████████████████████████▓▄▄▄µ   .████████
# ███████████H ╣███████████████████████████████████████████████████████▄ ║████████
# ████████████▌╚▓██████████████████████████Ñ  `╙▀█████████╨ "███████████▌╫████████
# █████████████ ╟▓████████████████████████▄▄µ,    ╙╣▓▓▓▓▀ ,▄▄████████████▓████████
# █████████████,:╠██████████████████████████████▄⌂      .▓████████████████████████
# █████████████▌ ╙▓███████████████████████████████▓     ██████████████████████████
# ██████████████  ╙▓█████████████████████████████████▓▓████▄██████████████████████
# ██████████████▌ "╩███████████████████████████████████▀▀▀▀▀▀▀████████████████████
# ███████████████   "▓███████████████▀▌▄▄▄▄▄███████████████████▄▄▓████████████████
# █████████████▀      ╨████████████Ñ▄████████████████████████████████¼████████████
# ████████████▀         ╙▀█▌████▀███████████████████████████████████▒█████████████
# ███████████.            `▀╦`╠▄▓████████████████▀▀▀▀╙░"`"▀▀██████████████████████
# ██████████φ]╦              ╓Å██▓█████████████▓▓██▓████▄▄▓▓▓████▀▀███████████████
# ████████▀Ü╦▓▓                "╫╨▀████████████████████████████▌▀` ███████████████
# ███████╓█╩█╫█▄                  ⁿ╙▀████████████████████████▀▀Γ  4███████████████
# ████████▓▀╫▀░],                   ^  ╙█████╫▓███▀█████M▀▀Ñ╙≈   ▀▌▓██████▓███████
# ████████▀▄>Φ▀╙Ü                        ``^`≈.╔┘╫M▀▀▓▀╟▀`        ▓██▓▓████▓██████
# ███████▀█ `~═¥╤ "                            ═^   "`.```         ╢▓██▄█▓████████
# ████████║▓▀`Å⌐ .  ~                                ,`             ╙█████████████
# ███████▌ █╝▀╨▀""      `                                            ,╫███████████
# ███████▀▄ ▌,▄M*                                                  ,█╣Ñ▀████████▓▓
# ██████▓▓▀H▐█  ^`        ≈                                       `▀▌ ,█¥▀████████
# ██████░▓▌╙ ▀▌▀╙Ç▄╦╨╬]                                            w`^▀,▄▓D▀██████
# █████▓▓▄`▓. █▌╙╙≈ ▄⌠¥ «⌂N╬▄]                                    ╝  ▄,╘████▓▀████
# █████▓▄╙▌▄╛  █Φ▀▀Ö⌠╠ ╣▓▄¼'╔⌐¡.                                 "Φ▄▓█└▓Ñwv▄▓█████
# ███▌▓▄█║╨,▄█▄ ████▌█╙▀▀▓M,▀,╗,▄  ╙N                         ▓▌  `Ñ*. ▀▀▌╬%Å▀╫███
# ██╙█▄▓▓██████ ╙█░▄▄ÅΦ,╙▌▓▌╔DM^`"" .▓@.                     ███⌐ ╓   "╤` ╓▄▀]╫███
# ▓▓▌███████████ ▀█▌`║▀▀██▓▓▌▓█╣╦w    "╙▄╥                  ╔███▌ , ╫▀KL ╙v,╚█▓███
# ██████████████▌ ▀█▄▄▓█▄▌▓▓▓║██▀██▀╙▌╔A▓╣▓Ü              ╓▄█████  "▓█▓▄Ü▀▄╙▓▓████
# ███████████████▄ ██▀▌▓█▀██▓⌂`Ñ` ▀  ╫Å▌▓ Å▄ Φw         ,▄███████▄`  ▀Ü ▀███▄█████
# ███▓████████████w ██▀╠▌▌ ▀█▓K▀╙   ▓φ║▄╙▓`╫▀╣▓▌╓     ,▄██████████   ▄█▀⌐,▓M▀▌████