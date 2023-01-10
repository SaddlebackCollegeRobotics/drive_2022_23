# drive_2022_23

# TODO:
- [x] **Resolve Odrive1 issue**
- [x] **Clean up src code**
- [x] **Implement/Design a left/right turn**
- [ ] **Create a server/client ros2 package for drive**

<br><br>

`ls_y`: Left stick `(y-axis only)`: Forward & Backwards movement
`rs_x`: Right stick `(x-axis only)`: Right/left turn *[To be implement]*

`l2`: L2 (hold) + `ls_y`: LStick (move): Move motors with speed
`l2`: L2 + `ls_y`: LStick (release): Cruise mode (Run on last given speed)

`r2`: R2 (Press) : Stop/discharge

<br><br>

Set up for drive right now:
```
{motor1}      {motor2}      {motor3}      {motor4}
   |              |            |              |
   \______________/            \______________/
       [Odrv0]                      [Odrv1]
          \                           /
        (Your hot garbage linux computer)  ------ [PS4 Controller]
```

If you're Sierra and you're a freaking weirdo who doesn't have Ubuntu on their machine:
```
{motor1}      {motor2}      {motor3}      {motor4}
   |              |            |              |
   \______________/            \______________/
       [Odrv0]                      [Odrv1]
          \                           /
            \______________________ /
                      |
                    (Loki)-------------------------[PS4 Controller]
                      |
                      | ssh
                      |
        (Sierra's hot garbage windows computer)
```

<br><br>
# Issues/Tasks:
## Resolve Odrive1 issue
Currently, if you run what we have in drive. Odrv1 will never spin, even though it calibrated properly its not going through the python script for some reason.

```py
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = int(ls_y * 10)
odrv0.axis1.controller.input_vel = int(ls_y * 10)

odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.controller.input_vel = int(ls_y * 10)
odrv1.axis1.controller.input_vel = int(ls_y * 10)
```
This snippet right here for example; only `odrv0` will pass and set the motors into closed loop control. Whilst it doesn't do that for `odrv1` (as seen by the setup/video we sent yesterday).

I think this has to do more with code and how it should be implemented. Previously, in our test script. We have to do something like this:
```py
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 10
odrv0.axis1.controller.input_vel = 10
if odrv_1:
    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.input_vel = 10
    odrv1.axis1.controller.input_vel = 10
```
With the function def, parameters set like to something like so:
```py
def test_motor (odrv_0, odrv_1=None):
```
Here's the (prev) full code:
```py
def test_motor (odrv_0, odrv_1=None):
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
```
Its worth investigating. And try to follow this format somewhat, see if that 1 if statement makes any difference. Haven't test this out yet but do try on Monday.

TL;DR. Change the current code to something like this. Maybe clean it up a bit too. Please reach out to me if you have any questions or is unclear on anything:
```py
def func(odrv_0, odrv_1=None):

    # some stuff here idk
    ...

    odrv0 = odrive.find_any(serial_number=odrv_0)
    if odrv_1:
        odrv1 = odrive.find_any(serial_number=odrv_1)

    # maybe a little magic here too
    ...

    while True:
          if controller is >0 or whatever then:
                odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                odrv0.axis0.controller.input_vel = int(ls_y * 10)
                odrv0.axis1.controller.input_vel = int(ls_y * 10)
            
                if odrv_1:
                    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    odrv1.axis0.controller.input_vel = int(ls_y * 10)
                    odrv1.axis1.controller.input_vel = int(ls_y * 10)

   cont...
```
I hope this makes sense.

<br>

## Clean up src code
Basically clean up the src code. For example: In `drive_2022_23/src/controller/gamepad_input.py` down at the bottom:
```py
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
```
I hate all these if's and else if's. If you can find a way to clean it up, that would be great. Thank you!!

And better organization is preferred too.

Find more if you can, thanks gorgeous(s) <-- idk the plural form of gorgeous

<br>

## Implement/Design a left/right turn
Right now we only have the forward/backward motion implemented. So we need to work on the turning aspect of the drive. This is pretty straightforward. 
Our rover does not have in-phase steering, so all 4 motors would face in the same direction. So to turn:
You can simply put 1 side of the rover, to spin at a higher speed than the other side, creating a drift effect, thereby "turning" it.

![Image](https://user-images.githubusercontent.com/49565505/211237222-53b98c69-ecfa-4a48-8cee-db47525d1aa4.png)

Draft ^ (idk I literally drew this as I'm eating). Let's say your `odrv0` is the left half of your rover, and `odrv1` is the right half. By turning `odrv0` at say, `15 m/s`, and `odrv1` is only at `7.5 m/s`. The high speed on the left side will probably cause the rover to steer more into the right side. So effectively, if you want to turn right, increase the speed of the left side, and vice versa.

UPDATE: I think to make good use of our time. One of you guys should devote sometime to calculating or finding out, what value/speed is the best to steer turn. Obviously don't burn the motors out. The values I mentioned are completely arbitrary, it could be `15m/s` & `0m/s` or one is half the other.

And I want to be able to do this motion through the right joystick of the controller. You can set this through `rs_x`. Cameron knows how it works so you can ask him, but effectively:
- If the right joystick is pushed to the right `rs_x` checks for analog input for the x-axis of it so `rs_x` is set to 1 now
```
[ _      1     _ ]
[ -1    0    *1* ]  <------ X-axis with your joystick pushed to the right
[ _     -1     _ ]
```
So TL;DR:
```py
if rs_x > 0:
    do right turn
    so odrv0 go brrrr and odrv1 go bruhbruhbruh
if rs_x < 0:
    d0 right turn
    odrv0 bruhrbruhbruh odrv1 brrrrrrrr
```

And this ^ should be in the `while TRUE` loop, where it keeps looking for update etc...

<br>

 ## Create a server/client ros2 package for drive
I forgot ros2 so ¯\\_(ツ)_/¯
Back to ros2 documentation :DDDDDDDDDD

But I think we need to create a server & client package. Where the server requests a package (in our case `loki` or any pi that is hooked up to drive). And the client, sends the package, which are just analog signal from the controller to the server. 

Here's the [doc](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

And here's a img from the doc so it looks like I know what I'm talking about:
![Image](https://user-images.githubusercontent.com/49565505/211238402-f9b259a5-6765-4b70-a7a8-c1ecbbee08e4.png)

This is not a priority task right now. But its nice to have someone to look into this, and just get started on it.

Here's a baby meme :DDD
![Image](https://user-images.githubusercontent.com/49565505/211238481-fb4cb691-a9af-4edb-b718-95c9b36f51b7.png)



