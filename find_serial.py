import subprocess


def get_all_odrives():

    odrivesSerNum = []

    usbDevices = str(subprocess.run('lsusb', capture_output=True).stdout).split('\\n')

    for device in usbDevices:
        if "ODrive" in device:
            serialNumber = device.split("Serial: ")[1]
            odrivesSerNum.append(serialNumber)

    return odrivesSerNum
        