# Author: Cameron Rosenthal @Supernova1114
# Description: A wrapper library for pygame's joystick module

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # Hide pygame welcome message
import pygame
import fileinput
import threading


# Init
pygame.init()
gamepadDict = {}
loop = True


# Quit program
def quit():
    global loop
    loop = False
    pygame.quit()
    print("Program Terminated")


# Get gamepad object
def getGamepad(index: int):
    if (pygame.joystick.get_count() > 0):
        return pygame.joystick.Joystick(index)
    else:
        return None


# Apply a deadzone to an axis
def applyDeadzone(value: float, deadzone: float):
    if (value < deadzone and value > -deadzone):
        return 0
    else:
        return value


# Get gamepad count
def getGamepadCount():
    return pygame.joystick.get_count()


# Call function if not None
def tryCallback(callback):
    if (callback is not None):
        callback()


# Get controller layout from config file
def parseGamepadLayout(gamepadName: str):
    
    gamepadLayout = []
    configFound = False
    file = fileinput.input(files ='gamepads.config')

    for line in file:

        if (line == '\n'):
            continue

        if (not configFound):

            if (line == gamepadName + '\n'):
                configFound = True
        else:
            if (line == 'END\n' or line == 'END'):
                break
            else:
                gamepadLayout.append(int(line.split(':')[1].strip()))

    file.close()
    return configFound, gamepadLayout


# Get gamepad layout from dictionary
def getGamepadLayout(instanceID: int):
    return gamepadDict[instanceID][1]


# Get axis tuples (example: left stick (x, y), right stick (x, y), triggers (l2, r2))
def getAxisGroup(gamepad, axis_1: int, axis_2: int):
    layout = getGamepadLayout(gamepad.get_instance_id())
    return gamepad.get_axis(layout[axis_1]), gamepad.get_axis(layout[axis_2])


# Get gamepad left and right triggers (l, r)
def getTriggers(gamepad, deadzone: float):
    (x, y) = getAxisGroup(gamepad, 11, 12)
    
    # Convert axis values from -1 to 1 to 0 to 2, and let deadzone be 0 to 1
    x = (x + 1) / 2
    y = (y + 1) / 2

    return applyDeadzone(x, deadzone), applyDeadzone(y, deadzone)
    

# Get gamepad left stick tuple (x,y)
def getLeftStick(gamepad, deadzone: float):
    (x, y) = getAxisGroup(gamepad, 13, 14)
    return applyDeadzone(x, deadzone), applyDeadzone(y, deadzone)


# Get gamepad right stick tuple (x,y)
def getRightStick(gamepad, deadzone: float):
    (x, y) = getAxisGroup(gamepad, 15, 16)
    return applyDeadzone(x, deadzone), applyDeadzone(y, deadzone)


# Get gamepad hat tuple (x,y)
def getHat(gamepad):
    return gamepad.get_hat(getGamepadLayout(gamepad.get_instance_id())[17])


# Tells if a button is pressed or not
def getButtonValue(gamepad, buttonIndex: int):
    return bool(gamepad.get_button(getGamepadLayout(gamepad.get_instance_id())[buttonIndex]) == 1)


# Handle new gamepad connections
def onGamepadConnected(event):

    gamepad = getGamepad(event.device_index)

    # Get controller layout from config file
    (configFound, buttonLayout) = parseGamepadLayout(gamepad.get_name())

    # Add gamepad and button layout to dictionary
    if (configFound):
        gamepadDict[gamepad.get_instance_id()] = (gamepad, buttonLayout)
        
    else:
        print("Controller Config Not Found!")
    
    print(f"Gamepad {gamepad.get_instance_id()} Connected", end = " | ")
    print(gamepad.get_name())


# Handle gamepad disconnections
def onGamepadRemoved(event):

    if (event.instance_id in gamepadDict.keys()):
        del gamepadDict[event.instance_id]
    
    print(f"Gamepad {event.instance_id} Disconnected")


# Handle gamepad button events
def onButtonEvent(event, callbackList):

    # Grab gamepad layout from dictionary
    layout = getGamepadLayout(event.instance_id)

    # Call function corresponding to button pressed
    for button, callback in zip (layout, callbackList):
        if (event.button == button):
            tryCallback(callback)
            break
    

# Handle gamepad hat events
def onHatMotion(event, callbackList):
    
    # Get gamepad hat
    layout = getGamepadLayout(event.instance_id)
    hat = gamepadDict[event.instance_id][0].get_hat(layout[17])

    # Handle X axis
    match hat[0]:
        case 1: # East
            tryCallback(callbackList[3])
        case -1: # West
            tryCallback(callbackList[2])

    # Handle Y axis
    match hat[1]:
        case 1: # North
            tryCallback(callbackList[0])
        case -1: # South
            tryCallback(callbackList[1])

    if (hat[0] == 0 and hat[1] == 0): # Hat is centered
        tryCallback(callbackList[4])


# Processes gamepad events and grants button callbacks
def run_event_loop(onButtonDownEvents = None, onButtonUpEvents = None, hatEvents = None):

    def async_wrapper():
    
        while loop:
            
            # Handle events
            for event in pygame.event.get():

                match event.type:
                    
                    # Handle quit event
                    case pygame.QUIT: 
                        quit()

                    # Get Button DOWN gamepad events
                    case pygame.JOYBUTTONDOWN:
                        if (onButtonDownEvents != None):
                            onButtonEvent(event, onButtonDownEvents)

                    # Get Button UP gamepad events
                    case pygame.JOYBUTTONUP:
                        if (onButtonUpEvents != None):
                            onButtonEvent(event, onButtonUpEvents)

                    # Handle gamepad hat events
                    case pygame.JOYHATMOTION:
                        if (hatEvents != None):
                            onHatMotion(event, hatEvents)
                    
                    # Handle new gamepad connections
                    case pygame.JOYDEVICEADDED: 
                        onGamepadConnected(event)

                    # Handle gamepad disconnections
                    case pygame.JOYDEVICEREMOVED: 
                        onGamepadRemoved(event)


    thread = threading.Thread(target=async_wrapper)
    thread.start()