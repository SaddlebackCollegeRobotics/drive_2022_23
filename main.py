from src.odrives.calibrate import *
import src.controller.gamepad_input as gmi
import subprocess



def get_all_odrives():

    odrivesSerialList = []

    usbDevices = str(subprocess.run(['lsusb', '-v'], capture_output=True).stdout).split('\\n')

    odriveFound = False
    for line in usbDevices:

        if "Odrive" in usbDevices:
            odriveFound = True

        # This will pull any device with a serial number 
        if odriveFound and "Serial" in usbDevices:
            odrivesSerialList.append(device[28:].strip())
            odrivesSerialList = list(filter(None, odrivesSerialList))
            odriveFound = False

    return odrivesSerialList



def north():
    print("North")

def west():
    print("West")

def south():
    print("South")
    gmi.rumbleAll(0, 0.3, 100)

def east():
    print("East")

def share():
    print("Share")

def options():
    print("Options")

def home():
    print("Home")

def l1():
    print("L1")

def r1():
    print("R1")

def l3():
    print("L3")

def r3():
    print("R3")


# Button UP callbacks
def northUp():
    print("North Up")

def westUp():
    print("West Up")

def southUp():
    print("South Up")

def eastUp():
    print("East Up")

def shareUp():
    print("Share Up")

def optionsUp():
    print("Options Up")

def homeUp():
    print("Home Up")

def l1Up():
    print("L1 Up")

def r1Up():
    print("R1 Up")

def l3Up():
    print("L3 Up")

def r3Up():
    print("R3 Up")


# Hat callbacks
def hatNorth():
    print("Hat North")

def hatSouth():
    print("Hat South")

def hatWest():
    print("Hat West")

def hatEast():
    print("Hat East")

def hatCentered():
    print("Hat Centered")


# Connection callbacks
def onGamepadConnect():
    ...

def onGamepadDisconnect():
    ...



AXIS_DEADZONE = 0.3 # Deadzone is 0 to 1 | Note: axis value will be 0 until you move past the deadzone



def eventHandler(odrv_0, odrv_1=None):
    buttonDownEvents = [
        north, west, south, east,
        share, options, home,                       
        l1, r1, l3, r3]

    buttonUpEvents = [
        northUp, westUp, southUp, eastUp, 
        shareUp, optionsUp, homeUp, 
        l1Up, r1Up, l3Up, r3Up]

    hatEvents = [hatNorth, hatSouth, hatWest, hatEast, hatCentered]                     # Set hat callbacks
    connectionEvents = [onGamepadConnect, onGamepadDisconnect]                          # Set connection callbacks
    gmi.run_event_loop(buttonDownEvents, buttonUpEvents, hatEvents, connectionEvents)   # Async loop to handle gamepad button events

    odrv0 = odrive.find_any(serial_number=odrv_0)       # Get odrive object
    odrv1 = odrive.find_any(serial_number=odrv_1)       # Get odrive object


    speed = 10


    while True:
        gp = gmi.getGamepad(0)                            # Get gamepad object

        (ls_x, ls_y) = gmi.getLeftStick(gp, AXIS_DEADZONE)  # Get left stick
        (rs_x, rs_y) = gmi.getRightStick(gp, AXIS_DEADZONE) # Get right stick
        (l2, r2) = gmi.getTriggers(gp, AXIS_DEADZONE)       # Get triggers
        (hat_x, hat_y) = gmi.getHat(gp)                     # Get hat
        
        if gmi.getButtonValue(gp, 1):
            ...

        # Left Stick, Forward/Backward Movement
        if l2 > 0:
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis0.controller.input_vel = int(ls_y * speed)
            odrv0.axis1.controller.input_vel = int(ls_y * speed)

            odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv1.axis0.controller.input_vel = int(ls_y * speed)
            odrv1.axis1.controller.input_vel = int(ls_y * speed)


        # Right Stick, Left/Right Movement
        # if rs_x > 0 and odrv1:
        #     odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv1.axis0.controller.input_vel += int(rs_x * 10)
        #     odrv1.axis1.controller.input_vel += int(rs_x * 10)
        # elif rs_x < 0:
        #     odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv0.axis0.controller.input_vel += int(rs_x * 10)
        #     odrv0.axis1.controller.input_vel += int(rs_x * 10)


# ====================================================================================================
        # TODO: For increasing/decreasing speed
        # if l2 > 0:
        #     print("L2")
        # if r2 > 0:
        #     print("R2")

        # TODO: Arm control
        # TODO: More optimal way to do this?
        # ! TEST MOTOR
# ====================================================================================================


        # Stop all motors if no analog input
        if r2 > 0:
            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            odrv0.axis1.requested_state = AXIS_STATE_IDLE
            odrv1.axis0.requested_state = AXIS_STATE_IDLE
            odrv1.axis1.requested_state = AXIS_STATE_IDLE

        if hat_x < 0:
            speed -= 5
            print("Current speed: ", speed)
        if hat_x > 0:
            speed += 5
            print("Current speed: ", speed)



if __name__ == "__main__":
    odrives = get_all_odrives()
    
    odrv0 = odrives[0]
    odrv1 = odrives[1]

    calibrate_all_motors(odrv0, odrv1)
    eventHandler(odrv0, odrv1)
    
