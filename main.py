"""
AUTHOR: NOT Parham Sharafoleslami
Project: Odrive Motor configuration
Date: NOT 11/31/2021
"""

import sys
import time
import odrive
#from odrive.enums import *
#import fibre.libfibre
#from enum import IntEnum
from loguru import logger

if __name__ == "__main__":

    #initialize odrv0
    odrv0 = odrive.find_any()

    #test function
    odrv0SerNum = str(hex(odrv0.serial_number)).upper()
    print(odrv0SerNum.replace('0X',''))

    