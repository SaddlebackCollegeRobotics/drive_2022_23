# Author: Cameron Rosenthal @Supernova1114
# Description: A wrapper library for pygame's joystick module

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide" # Hide pygame welcome message
import pygame
import threading
import signal


# Global variables and init
pygame.init()
gamepadDict = {}
loop = True
thread = None

# Set default config file path (gamepads.config in same directory as this file)
configFile = os.path.dirname(os.path.realpath(__file__)) + "/gamepads.config"


# Signal handler for Ctrl+C
def signalHandler(signal, frame):
    quit()

signal.signal(signal.SIGINT, signalHandler)


# Set new config file path
def setConfigFile(path: str):
    global configFile
    configFile = path


# Quit program
def quit():
    global loop
    loop = False

    # Wait for thread to finish
    if (thread != None):
        thread.join()

    pygame.quit()
    print("\nGamepad Input Terminated Safely\n")
    exit(0)


# Get gamepad object
def getGamepad(index: int):
    values = list(gamepadDict.values())
    return values[index][0] if index < values.__len__() else None


# Apply a deadzone to an axis (value is zero until past deadzone)
def applyDeadzone(value: float, deadzone: float):
    return 0 if (value < deadzone and value > -deadzone) else value


# Get gamepad count
def getGamepadCount():
    return gamepadDict.__len__()


# Call function if not None
def tryCallback(callback):
    if (callback != None):
        callback()


# Get controller layout from config file
def parseGamepadLayout(gamepadName: str):
    
    gamepadLayout = []
    configFound = False

    # Open config file
    try:
        file = open(configFile)
    except:
        print("Error: Config file not found")
        return None

    # Parse config file
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

    if (not configFound):
        print("Error: Gamepad config not found")
    
    return gamepadLayout if configFound else None


# Get gamepad layout from dictionary
def getGamepadLayout(instanceID: int):
    return gamepadDict[instanceID][1] if instanceID in gamepadDict.keys() else None
    

# Get axis tuples (example: left stick (x, y), right stick (x, y), triggers (l2, r2))
def getAxisGroup(gamepad, axis_1: int, axis_2: int):

    layout = getGamepadLayout(gamepad.get_instance_id())

    if (layout != None):
        return gamepad.get_axis(layout[axis_1]), gamepad.get_axis(layout[axis_2])
    else:
        return None


# Get gamepad left and right triggers (l, r)
def getTriggers(gamepad, deadzone: float):

    if (gamepad == None):
        return (0, 0)

    axisGroup = getAxisGroup(gamepad, 11, 12)
    if (axisGroup == None):
        return (0, 0)

    # Convert axis values from -1 to 1 to 0 to 2, and let deadzone be 0 to 1
    x = (axisGroup[0] + 1) / 2
    y = (axisGroup[1] + 1) / 2

    return applyDeadzone(x, deadzone), applyDeadzone(y, deadzone)


# Get gamepad left stick tuple (x,y)
def getLeftStick(gamepad, deadzone: float):

    if (gamepad == None):
        return (0, 0)

    axisGroup = getAxisGroup(gamepad, 13, 14)
    if (axisGroup == None):
        return (0, 0)

    return applyDeadzone(axisGroup[0], deadzone), applyDeadzone(axisGroup[1], deadzone)
    

# Get gamepad right stick tuple (x,y)
def getRightStick(gamepad, deadzone: float):

    if (gamepad == None):
        return (0, 0)

    axisGroup = getAxisGroup(gamepad, 15, 16)
    if (axisGroup == None):
        return (0, 0)

    return applyDeadzone(axisGroup[0], deadzone), applyDeadzone(axisGroup[1], deadzone)
    

# Get gamepad hat tuple (x,y)
def getHat(gamepad):

    if (gamepad == None):
        return (0, 0)

    layout = getGamepadLayout(gamepad.get_instance_id())
    return (0, 0) if layout == None else gamepad.get_hat(layout[17])


# Tells if a button is pressed or not
# Button index corresponds to order of button layout in config file
def getButtonValue(gamepad, buttonIndex: int):

    if (gamepad == None):
        return False

    layout = getGamepadLayout(gamepad.get_instance_id())
    return False if layout == None else bool(gamepad.get_button(layout[buttonIndex]) == 1) 


# Rumble all gamepads
# Low frequency is the heavy rumble (0 to 1)
# High frequency is the light rumble (0 to 1)
def rumbleAll(low_frequency: float, high_frequency: float, duration: int):
    for gamepad in gamepadDict.values():
        gamepad[0].rumble(low_frequency, high_frequency, duration)


# Stop rumble on all gamepads
def stopRumbleAll():
    for gamepad in gamepadDict.values():
        gamepad[0].stop_rumble()


# Handle new gamepad connections
def onGamepadConnected(event):

    # Get gamepad from pygame
    gamepad = pygame.joystick.Joystick(event.device_index)

    # Get controller layout from config file
    buttonLayout = parseGamepadLayout(gamepad.get_name())

    # Add gamepad and button layout to dictionary
    if (buttonLayout != None):
        gamepadDict[gamepad.get_instance_id()] = (gamepad, buttonLayout)

    print(f"Gamepad {gamepad.get_instance_id()} Connected", end = " | ")
    print(gamepad.get_name())    


# Handle gamepad disconnections
def onGamepadRemoved(event):

    id = event.instance_id

    if (id in gamepadDict.keys()):
        del gamepadDict[id]
    
    print(f"Gamepad {id} Disconnected")


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
    if hat[0] == 1:
        tryCallback(callbackList[3])
    else:
        tryCallback(callbackList[2])

    # Handle Y axis
    if hat[1] == 1:
        tryCallback(callbackList[0])
    else:
        tryCallback(callbackList[1])

    if (hat[0] == 0 and hat[1] == 0): # Hat is centered
        tryCallback(callbackList[4])


# Processes gamepad events and grants button callbacks
def run_event_loop(onButtonDownEvents = None, onButtonUpEvents = None, hatEvents = None, connectionEvents = None):

    def async_wrapper():
    
        while loop:
            
            # Handle events
            for event in pygame.event.get():

                if event.type == pygame.QUIT: 
                    quit()
                elif event.type == pygame.JOYBUTTONDOWN:
                    if (onButtonDownEvents != None):
                        onButtonEvent(event, onButtonDownEvents)
                elif event.type == pygame.JOYBUTTONUP:
                    if (onButtonUpEvents != None):
                        onButtonEvent(event, onButtonUpEvents)
                elif event.type == pygame.JOYHATMOTION:
                    if (hatEvents != None):
                        onHatMotion(event, hatEvents)
                elif event.type == pygame.JOYDEVICEADDED: 
                    onGamepadConnected(event)
                    if (connectionEvents != None):
                        tryCallback(connectionEvents[0])
                elif event.type == pygame.JOYDEVICEREMOVED: 
                    onGamepadRemoved(event)
                    if (connectionEvents != None):
                        tryCallback(connectionEvents[1])


    global thread
    thread = threading.Thread(target=async_wrapper)
    thread.start()