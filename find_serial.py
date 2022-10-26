import subprocess

#==================Purpose and approach====================
# We need to find every odrive that is connected to the computer
#  without simply using odrive.find_any() because it will only connect to 
#  the first one it finds. It could also result in an error where
#  during calibration the motor that moves jumps to a different odrive,
#  which doesn't allow proper reductive testing.
#
#
# The approach is to use the serial numbers of the odrives so that
#  the find_any() function singles out the selected odrive and connects
#  solely to that one. The old way was to get the user to input every serial
#  number of each odrive that was connected to the machine. As some would
#  imagine, this method made our lazy butts very angry. 
# 
# If you use lsusb in your terminal, it will list every device that is currently
#  connected to the machine, including its serial number. The following code 
#  uses lsusb in the terminal, captures the output, and parses through to obtain 
#  the serial numbers. This is turned into a list and returned.
#==========================================================
def get_all_odrives():

    odrivesSerNum = []

    usbDevices = str(subprocess.run(['lsusb', '-v'], capture_output=True).stdout).split('\\n')

    for device in usbDevices:
        if "ODrive" in device:
            serialNumber = device.split("Serial: ")[1]
            odrivesSerNum.append(serialNumber)

    return odrivesSerNum
        