import subprocess

#if __name__ == "__main__":

def get_all_odrives():
    usbDevices = str(subprocess.run('lsusb', capture_output=True).stdout).split('\\n')

    for device in usbDevices:
        if "ODrive" in device:
            serialNumber = device.split("Serial: ")[1]
            print(serialNumber)

        