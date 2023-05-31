""" New calibration file becaue the old one is shit """
from threading import Thread
import time

import odrive
from odrive.enums import AxisState, EncoderId, MotorType


def get_all_odrives() -> tuple[str, str]:
    """Get serial numbers of all connected ODrives
    
    Returns:
        tuple[str, str]: (left odrive, right odrive)

    TODO: remove hard coded serial numbers
    NOTE: Run find_devpath.bash to get odrive serial number.
    """
    return ('207937815753','366B385A3030')


def calibrate_all_motors(odrv_left: str, odrv_right: str) -> None:
    """Calibrate all NEO motors attached to odrives
    
    Args:
        odrv_left (str): Serial number of left odrive
        odrv_right (str): Serial number of right odrive
    """
    print(f'[INFO] Odrive 0: {odrv_left} \nOdrive 1: {odrv_right} \n\n\n')    

    print('[INFO] Creating threads for Odrives')
    threads = [Thread(target=_calibrate_motor, args=(odrv_left, 0)),
               Thread(target=_calibrate_motor, args=(odrv_left, 1)), 
               Thread(target=_calibrate_motor, args=(odrv_right, 0)),
               Thread(target=_calibrate_motor, args=(odrv_right, 1))] 

    # start all motor calibrations
    for thread in threads:
        thread.start()
        print('[INFO] -- Odrive Thread started --')
        time.sleep(1)

    # wait for all motor calibrations to finish
    for thread in threads:
        thread.join()
        print('[SUCCESS] Odrive successfully callibrated')

    print("[SUCCES] All calibrations complete, ready to rove")

# TODO check for thermistor
def _calibrate_motor(odrv_serial_num: str, axis_num: int) -> None:
    """ Configure Neo motors/ Odrive with Odrive documenation settings 
    
    Args:
        odrv_serial_num (str): serial number of odrive to calibrate
        axis_num (int): axis number of motor on odrive, usually 0 or 1
    """
    NUM_BATTERY_CELLS = 3
    MIN_CELL_VOLTAGE = 3.3  # [V]
    MAX_CELL_VOLTAGE = 4.25  # [V]
    MAX_BATTERY_DISCHARGE_CURRENT = 143.75  # [A]
    MAX_BATTERY_CHARGING_CURRENT = 115.0  # [A]
    NUM_PERMANENT_MAGNETS = 7

    MOTOR_KV = 473  # [rotations/V]
    TORQUE_CONSTANT = 8.27 / MOTOR_KV  # [Nm]

    MAX_CONTINUOUS_MOTOR_CURRENT = 40  # [A]

    # 20% voltage buffer to be safe
    MAX_CALIB_VOLTAGE = 11.1/2 * 0.8 # [V]
    MAX_PEAK_MOTOR_CURRENT = 100  # [V]
    MAX_MOTOR_VEL = 94.6  # [turns/s]
    
    # Find Odrive
    info_str = f'[{odrv_serial_num}:{axis_num}]'
    
    odrv = odrv.find_any(serial_number=odrv_serial_num)
    vbus_voltage = odrv.vbus_voltage

    # odrv = erase_config(odrv_num, clear) TODO is this necessary?
    print('[SUCCESS] Odrive found!')
    print(f'{info_str} voltage: {vbus_voltage}')

    odrv_axis = getattr(odrv, f'axis{axis_num}')
    odrv_hall_encoder = getattr(odrv, f'hall_encoder{axis_num}')

    # === Configure Odrive POWER ===
    print('[CONFIGURING] limits ...')
    # Voltage Limits
    odrv.config.dc_bus_undervoltage_trip_level = MIN_CELL_VOLTAGE * NUM_BATTERY_CELLS
    odrv.config.dc_bus_overvoltage_trip_level = MAX_CELL_VOLTAGE * NUM_BATTERY_CELLS

    # Current Limits
    odrv.config.dc_max_positive_current = MAX_BATTERY_DISCHARGE_CURRENT
    odrv.config.dc_max_negative_current = MAX_BATTERY_CHARGING_CURRENT

    # === Configure Odrive MOTOR ===
    print('[CONFIGURING] motor ...')

    odrv_axis.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv_axis.config.motor.pole_pairs = NUM_PERMANENT_MAGNETS
    odrv_axis.config.motor.torque_constant = TORQUE_CONSTANT
    odrv_axis.config.motor.calibration_current = MAX_CONTINUOUS_MOTOR_CURRENT / 2
    odrv_axis.config.motor.resistance_calib_max_voltage = MAX_CALIB_VOLTAGE
    odrv_axis.config.motor.calibration_lockin.current = MAX_CONTINUOUS_MOTOR_CURRENT / 2
    
    # === Start Calibration and Save Configuration ===
    print('[CALIBRATING] ...')
    odrv_axis.requested_state = AxisState.MOTOR_CALIBRATION
    print('[SUCCESS] saving to config')
    odrv.save_configuration()

    # === Set Limits ===
    # Current limit
    print('[CALIBRATING] limits ...')
    odrv_axis.config.motor.current_soft_max = MAX_CONTINUOUS_MOTOR_CURRENT
    odrv_axis.config.motor.current_hard_max = MAX_PEAK_MOTOR_CURRENT

    # Velocity limit
    odrv_axis.controller.config.vel_limit = MAX_MOTOR_VEL

    # === Encoder Configuration ===
    print('[CALIBRATING] encoder ...')
    odrv_axis.config.encoder_bandwidth = 100  # default
    odrv_hall_encoder.config.enabled = True

    encoder_type = EncoderId.HALL_ENCODER0 if axis_num == 0 else EncoderId.HALL_ENCODER0
    odrv_axis.config.load_encoder = encoder_type
    odrv_axis.config.commutation_encoder = encoder_type
    print('[INFO] saving to config')
    odrv.save_configuration()
    # Wait for ODrive to reboot #TODO how??
    print('[CALIBRATING] encoder hall polarity')
    odrv_axis.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION
    # Wait for motor to stop  #TODO how??
    print('[CALIBRATING] encoder hall phase')
    odrv_axis.requested_state = AxisState.ENCODER_HALL_PHASE_CALIBRATION
    # Wait for motor to stop  #TODO how??
    print('[INFO] Calibration finished')
