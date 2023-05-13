# ========================================================================================
# Author:   Jasper Doan     @wluxie
# Date:     02/01/2023
# ========================================================================================
# Description:
#   I don't know what this does, but it's important so don't touch it
#       jk you can touch it, but don't break it
#   Though I don't even know how you would break it, since its just printing stuff
#       and half of it does nothing
#   This is useful if you want to make functional calls from your controller
#       like rumble, or changing the color of the light bar, or with buttons pressed
#   We didn't ended up using this bc we only need 6 inputs, but it's here if you want 
#       to use it
# ========================================================================================

class Buttons:
    def north():
        print("North")

    def west():
        print("West")

    def south():
        print("South")
        # gmi.rumbleAll(0, 0.3, 100)

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
        ...

    def westUp():
        ...

    def southUp():
        ...

    def eastUp():
        ...

    def shareUp():
        ...

    def optionsUp():
        ...

    def homeUp():
        ...

    def l1Up():
        ...

    def r1Up():
        ...

    def l3Up():
        ...

    def r3Up():
        ...


    # Hat callbacks
    def hatNorth():
        ...

    def hatSouth():
        ...

    def hatWest():
        print("🦽 Going backwards!! 🦽")

    def hatEast():
        print("💨 Ramp up Speed!! 💨")

    def hatCentered():
        ...


    # Connection callbacks
    def onGamepadConnect():
        ...

    def onGamepadDisconnect():
        ...
