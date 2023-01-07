import odrive
from odrive.enums import *


# def test_motor (odrv_0, odrv_1=None, odrv_2=None):
def test_motor(odrv_0, odrv_1=None):

    odrv0 = odrive.find_any(serial_number=odrv_0)


    print("Enter velocity: ")
    velVal = input()


    while True:
        try:
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            odrv0.axis0.controller.input_vel = int(velVal)
            odrv0.axis1.controller.input_vel = int(velVal)

            if odrv_1:
                odrv1 = odrive.find_any(serial_number=odrv_1)

                odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

                odrv1.axis0.controller.input_vel = int(velVal)
                odrv1.axis1.controller.input_vel = int(velVal)
                

            # if odrv_2:
            #     odrv2 = odrive.find_any(serial_number=odrv_2)

            #     odrv2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            #     odrv2.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            #     odrv2.axis0.controller.input_vel = int(velVal)
            #     odrv2.axis1.controller.input_vel = int(velVal)

            print("Enter Velocity: ")
            velVal = input()
        
        except ValueError:
            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            odrv0.axis1.requested_state = AXIS_STATE_IDLE

            if odrv_1:
                odrv1.axis0.requested_state = AXIS_STATE_IDLE
                odrv1.axis1.requested_state = AXIS_STATE_IDLE

            # if odrv_2:
            #     odrv2.axis0.requested_state = AXIS_STATE_IDLE
            #     odrv2.axis1.requested_state = AXIS_STATE_IDLE

            break