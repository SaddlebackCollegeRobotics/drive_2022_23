# Gamepad Input API
A wrapper for Pygame's joystick module
<br>
Author: Cameron Rosenthal
<br>

Pygame: https://github.com/pygame/pygame

Tested with Pygame 2.1.2

#### Linux install Pygame: 

    pip install pygame

#### Features
- Controller hot-plug support.
- Support for matching button and axis layouts across different controllers.
- Function callbacks for button presses (Async), in addition to button value reading.
- Easy access to grouped joystick axis data. (Ex: (x,y) for Left Stick).
- Axis deadzones.
- Muliple-controller support.
#### Examples and Gamepad tester
- Example use of API is shown in api_example.py
- Pygame's gamepad tester is controller_test.py
#### Creating a controller configuration
- Use controller_test.py to see button and axis ID's, and controller name.
- Open gamepads.config and assign correct ID's to each button / axis.
- Make sure controller name is correct.
