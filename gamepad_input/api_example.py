# Author: Cameron Rosenthal @Supernova1114
# Description: Example use of gamepad_input API

import gamepad_input


# Button DOWN callbacks
def north():
    print("North")

def west():
    print("West")

def south():
    print("South")
    gamepad_input.rumbleAll(0, 0.3, 100)

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

# Main function
def main():

    # Set config file to use to lookup gamepad layouts.
    # If you don't set this, default file path will be
    # gamepads.config in the same directory as this file

    # gamepad_input.setConfigFile("path/to/config/file")

    # To get directory of current script:
    # os.path.dirname(os.path.realpath(__file__))

    # Set button callbacks
    buttonDownEvents = [north, west, south, east, share, options, home, l1, r1, l3, r3]
    buttonUpEvents = [northUp, westUp, southUp, eastUp, shareUp, optionsUp, homeUp, l1Up, r1Up, l3Up, r3Up]

    # Note: You can set a callback to 'None' if you don't want to use it

    # Set hat callbacks
    hatEvents = [hatNorth, hatSouth, hatWest, hatEast, hatCentered]

    # Set connection callbacks
    connectionEvents = [onGamepadConnect, onGamepadDisconnect]

    # Async loop to handle gamepad button events
    gamepad_input.run_event_loop(buttonDownEvents, buttonUpEvents, hatEvents, connectionEvents)

    # Note: callbacks are by default set to 'None' if not specified as arguments
    # Ex: gamepad_input.run_event_loop(buttonDownEvents, buttonUpEvents)
    # Ex: gamepad_input.run_event_loop(buttonDownEvents, None, None)

    # You can set individual callbacks to None inside callback arrays if you do not need them
    # Ex: buttonDownEvents = [north, west, None, east]

    # How to get gamepad object and read axis values
    axis_deadzone = 0.3 # Deadzone is 0 to 1
    # Note: axis value will be 0 until you move past the deadzone

    # Rumble all gamepads
    # Low frequency is the heavy rumble (0 to 1)
    # High frequency is the light rumble (0 to 1)
    # def rumbleAll(low_frequency: float, high_frequency: float, duration: int)

    # gamepad_input.rumbleAll(0, 0.3, 100)

    # Stop rumble on all gamepads
    # gamepad_input.stopRumbleAll()

    # Safely quit program
    # gamepad_input.quit()
    
    while True:
        
        # Get gamepad object by index
        gamepad = gamepad_input.getGamepad(0)

        # Get left stick axis values
        (ls_x, ls_y) = gamepad_input.getLeftStick(gamepad, axis_deadzone)
        (rs_x, rs_y) = gamepad_input.getRightStick(gamepad, axis_deadzone)

        # Get trigger axis values (axis goes from -1 to 1)
        (l2, r2) = gamepad_input.getTriggers(gamepad, axis_deadzone)

        # Get hat axis values
        # You can also use the hat callbacks from the run_event_loop() function
        (hat_x, hat_y) = gamepad_input.getHat(gamepad)

        # If you want to check if a button is pressed, you can use this
        # or use the callback functions method from the run_event_loop() function
        # Button index corresponds to the order of preset 
        if gamepad_input.getButtonValue(gamepad, 1):
            ...

        # Print axis values
        if ls_x > 0:
            print("LS Right")
        elif ls_x < 0:
            print("LS Left")

        if ls_y > 0:
            print("LS Down")
        elif ls_y < 0:
            print("LS Up")

        if rs_x > 0:
            print("RS Right")
        elif rs_x < 0:
            print("RS Left")

        if rs_y > 0:
            print("RS Down")
        elif rs_y < 0:
            print("RS Up")

        if l2 > 0:
            print("L2")

        if r2 > 0:
            print("R2")

        if hat_x > 0:
            print("Hat East")

        if hat_x < 0:
            print("Hat West")

        if hat_y < 0:
            print("Hat South")

        if hat_y > 0:
            print("Hat North")


# Run the program
if __name__ == "__main__":
    main()