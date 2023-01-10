from callback_api import *
from src.odrives.calibrate import *
import src.controller.gamepad_input as gmi
import subprocess



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

    speed = 10

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

        # Left Stick, Forward/Backward Movement
        if l2 > 0 and odrv_1:
            print("Left Stick: ", ls_x, ls_y)
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis0.controller.input_vel = int(ls_y * speed)
            odrv0.axis1.controller.input_vel = int(ls_y * speed)
            odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv1.axis0.controller.input_vel = int(ls_y * speed)
            odrv1.axis1.controller.input_vel = int(ls_y * speed)

        # Stop all motors if no analog input
        if (r2 > 0 or ls_y == 0) and odrv_1:
            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            odrv0.axis1.requested_state = AXIS_STATE_IDLE
            odrv1.axis0.requested_state = AXIS_STATE_IDLE
            odrv1.axis1.requested_state = AXIS_STATE_IDLE

        # Ramp up input
        if hat_x < 0 and speed != -20:
            speed -= 5
            print("Current speed: ", speed)
        if hat_x > 0 and speed != 20:
            speed += 5
            print("Current speed: ", speed)
            
        # Right Stick, Left/Right Movement
        # if rs_x > 0 and odrv1:
        #     odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv1.axis0.controller.input_vel += int(rs_x * 10)
        #     odrv1.axis1.controller.input_vel += int(rs_x * 10)
        # elif rs_x < 0 and odrv1:
        #     odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     odrv0.axis0.controller.input_vel += int(rs_x * 10)
        #     odrv0.axis1.controller.input_vel += int(rs_x * 10)

# ====================================================================================================
        # if l2 > 0:
        #     print("L2")
        # if r2 > 0:
        #     print("R2")

        # TODO: Arm control
        # TODO: More optimal way to do this?
        # ! TEST MOTOR
# ====================================================================================================




if __name__ == "__main__":
    odrives = get_all_odrives()
    # Odrive 0:  366B385A3030 
    # Odrive 1:  365F385E3030
    odrv0 = odrives[0]
    odrv1 = odrives[1]

    calibrate_all_motors(odrv0, odrv1)
    eventHandler(odrv_0=odrv0, odrv_1=odrv1)
    