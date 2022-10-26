"""
AUTHOR: Max Rehm & Matin Qurbanzadeh
Project: Odrive Motor configuration
Date: 10/19/2022
"""

from calibrate import calib_motor
from configure import config_motor
from find_serial import get_all_odrives
from loguru import logger



if __name__ == "__main__":

    #finding odrive
    odrives = get_all_odrives()

    logger.debug("Would you like to test each motor individually after each is calibrated[Y], if not just hit enter...")
    logger.debug("Note: You will still be prompted to test all motors at once.")
    motorTestCMD = input().upper()

    
    #loop through each odrive that is connected and configure/calibrate each axis_num.
    for odrvSerial in odrives:
        config_motor(odrvSerial, 0, True)
        calib_motor(odrvSerial, 0, motorTestCMD)

        config_motor(odrvSerial, 1, False)
        calib_motor(odrvSerial, 1, motorTestCMD)

#I hate you