from callback_api import *
from src.odrives.calibrate import *
import src.controller.gamepad_input as gmi
import subprocess



AXIS_DEADZONE = 0.3 # Deadzone is 0 to 1 | Note: axis value will be 0 until you move past the deadzone
MIN_SPEED = -20
MAX_SPEED = 20
CREMENT = 5



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

    speed = 5
    rampLVel, rampRVel = 0, 0
    rampVel = [rampLVel, None, rampRVel]

    while True:
        odrv0 = odrive.find_any(serial_number=odrv_0)       # Get odrive object
        if odrv_1:
            odrv1 = odrive.find_any(serial_number=odrv_1)   # Get odrive object

        gp = gmi.getGamepad(0)                              # Get gamepad object

        (ls_x, ls_y) = gmi.getLeftStick(gp, AXIS_DEADZONE)  # Get left stick
        (rs_x, rs_y) = gmi.getRightStick(gp, AXIS_DEADZONE) # Get right stick
        (l2, r2) = gmi.getTriggers(gp, AXIS_DEADZONE)       # Get triggers
        (hat_x, hat_y) = gmi.getHat(gp)                     # Get hat
        
        if gmi.getButtonValue(gp, 1):
            ...

        velocity = ls_y * speed
        rampVel[int(rs_x) + 1] = (speed * abs(rs_x)) / 1.5
            

        # Left Stick, Forward/Backward Movement
        if l2 > 0 and odrv_1:
            print("Left Stick: ", ls_x, ls_y, "\tRight Stick: ", rs_x, rs_y)
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis0.controller.input_vel = int(velocity + rampLVel)
            odrv0.axis1.controller.input_vel = int(velocity + rampLVel)
            odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv1.axis0.controller.input_vel = int(velocity + rampRVel)
            odrv1.axis1.controller.input_vel = int(velocity + rampRVel)

        # Stop all motors if no analog input
        if (ls_y == 0 and ls_x == 0) and odrv_1:
            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            odrv0.axis1.requested_state = AXIS_STATE_IDLE
            odrv1.axis0.requested_state = AXIS_STATE_IDLE
            odrv1.axis1.requested_state = AXIS_STATE_IDLE

        # Ramp up/down input
        if hat_x < 0 and speed != MIN_SPEED:
            speed -= CREMENT
            print("Current speed: ", speed)
        if hat_x > 0 and speed != MAX_SPEED:
            speed += CREMENT
            print("Current speed: ", speed)
            

# ====================================================================================================
        # TODO: Arm control
# ====================================================================================================




if __name__ == "__main__":
    odrives = get_all_odrives()
    # Odrive 0:  366B385A3030 
    # Odrive 1:  365F385E3030
    odrv0 = odrives[0]
    odrv1 = odrives[1]

    calibrate_all_motors(odrv0, odrv1)
    eventHandler(odrv_0=odrv0, odrv_1=odrv1)
    