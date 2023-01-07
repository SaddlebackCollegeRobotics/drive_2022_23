import gamepad_input

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