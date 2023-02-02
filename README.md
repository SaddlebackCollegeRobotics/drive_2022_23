# drive_2022_23

## Description
- Drive repository for the 2022-23 season of URC by SC Robotics. This repository contains the code for driving the rover through using a **PS4** controller.
- The drive system is written in *Python* and uses **ROS2** for communication between the rover and the controller.
- The drive system uses **2 Odrive** drivers to control the motors on the rover. Totalling **4 NEOs Brushless DC Motors**.
- We also have a local development environment that can be used to test the code before deploying it to the rover. It is on the branch `local-development`.

## Installation and Setup
### Setup
*Local development environment:*
```
{motor1}      {motor2}      {motor3}      {motor4}
   |              |            |              |
   \______________/            \______________/
       [Odrv0]                      [Odrv1]
          \                           /
        (Your hot garbage linux computer)  ------ [PS4 Controller]
```
*Remote/Competition enviroment:*
```
{motor1}      {motor2}      {motor3}      {motor4}
   |              |            |              |
   \______________/            \______________/
       [Odrv0]                      [Odrv1]
          \                           /
            \______________________ /
                      | 
                    (Loki)
                      |     //ssh through ymir's terminal*
                      |
                    (Ymir)  // Gate way of the rover
                      |
                      |     //ssh through telecom*
                      |
        (Your hot garbage windows computer) -------------[PS4 Controller]
```
### Installation:
``` bash
# Clone the repository, where <path> is the path to the directory you want to clone the repository into
git clone https://github.com/SC-Robotics-2021/drive_2022_23.git

# Install the dependencies
(figure that our on your own, I'm not your mom)
(hint: check out our spaghetti code, and look at the imports)

# Run the code
# In terminal 1
source /opt/ros/foxy/setup.bash
cd ~/drive_2022_23/src/driver/
make
. install/setup.bash
make pub

# In terminal 2
. install/setup.bash
make sub
```

## Overview
- The code is split into two parts: the publisher and the subscriber.
### Publisher:
- The publisher is responsible for reading the input from the PS4 controller and publishing it to the subscriber.
- This is the ROS2 node that publishes the controller input to the ROS2 topic 'controls'.
    + The topic is a Float64MultiArray with 6 elements (in a [3x2] matrix, but stored as List)
```
    ___                         ___
    | left_stick_x    left_stick_y  |
    | right_stick_x   right_stick_y |
    | left_trigger    right_trigger |
    |___                         ___|
```

- The values are normalized to be between -1 and 1, deadzoned to be 0 if the value is less than AXIS_DEADZONE, and are rounded to 2 decimal places. The values are published at a rate of 10 Hz
- Where the message are published are as follows:
```
- msg :: Float64MultiArray[6]
    + msg.data[0] :: left_stick_x        + msg.data[1] :: left_stick_y
    + msg.data[2] :: right_stick_x       + msg.data[3] :: right_stick_y
    + msg.data[4] :: left_trigger        + msg.data[5] :: right_trigger
```

Team's garbage code:
```python
def timer_callback(self):
    gp = gmi.getGamepad(0)

    (ls_x, ls_y) = gmi.getLeftStick(gp, AXIS_DEADZONE)  # Get left stick
    (rs_x, rs_y) = gmi.getRightStick(gp, AXIS_DEADZONE) # Get right stick
    (l2, r2) = gmi.getTriggers(gp, AXIS_DEADZONE)       # Get triggers

    msg = Float64MultiArray()                           # Create message

    # Set message data
    msg.data = [float(ls_x), float(ls_y), float(rs_x), float(rs_y), float(l2), float(r2)]

    self.publisher_.publish(msg)                        # Publish that sucker

    # Print for debugging
    print('😤😤 SENDING [LS: (%.2f, %.2f) | RS: (%.2f, %.2f) | LT: %.2f | RT: %.2f] 😤😤' % (ls_x, ls_y, rs_x, rs_y, l2, r2))
```

### Subscriber:
- The subscriber is responsible for reading the controller input from the publisher and sending it to the rover (or rather to the odrives).
- This is the ROS2 node that subscribes to the ROS2 topic 'controls' and sends the controller input to the odrives.
    + The topic is a Float64MultiArray with 6 elements (in a [3x2] matrix, but stored as List)
```python
def listener_callback(self, msg):
    (ls_x, ls_y, rs_x, rs_y, l2, r2) = msg.data             # Unpack message data

    print('😫😫 RECEIVED [LS: (%.2f, %.2f) | RS: (%.2f, %.2f) | LT: %.2f | RT: %.2f] 😫😫' % (ls_x, ls_y, rs_x, rs_y, l2, r2))
```
Then data would just be fed into the team odrive's controller code. Math looks something like this *(but not the actual thing bc we don't want to give away our secrets)*:
```python
odrv0 = odrive.find_any(serial_number=odrv_0)       # Get odrive object
if odrv_1:
    odrv1 = odrive.find_any(serial_number=odrv_1)   # Get odrive object

velocity = ls_y * speed
ramp = abs(rs_x)/1.5 + 1
(rampLVel, rampRVel) = (1, 1)

if rs_x < 0:
    rampRVel = ramp
elif rs_x > 0:
    rampLVel = ramp

# Lambda expressions
reqVel = lambda rvel: int(velocity * rvel)  # Requested velocity
rndS = lambda x: round(x, 2)                # Formatting stick values

# Left Stick, Forward/Backward Movement
if l2 > 0 and odrv_1:
    print("Left Stick:", (rndS(ls_x), rndS(ls_y)), "\tRight Stick:", (rndS(rs_x), rndS(rs_y)))
    print("Left Speed:", reqVel(rampLVel), "\t\tRight Speed:", reqVel(rampRVel))
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.input_vel = reqVel(rampLVel)
    odrv0.axis1.controller.input_vel = reqVel(rampLVel)
    odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.input_vel = reqVel(rampRVel)
    odrv1.axis1.controller.input_vel = reqVel(rampRVel)

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
```

## Contributors
| [![wluxie](https://avatars.githubusercontent.com/u/49565505?v=4)](https://github.com/wluxie) | [![Supernova1114](https://avatars.githubusercontent.com/u/55326068?v=3)](https://github.com/Supernova1114) | [![max](https://avatars.githubusercontent.com/u/111012399?v=4)](https://github.com/12max345) | ![matin](https://avatars.githubusercontent.com/u/61672425?v=4) | ![sierra](https://avatars.githubusercontent.com/u/86510695?v=4) | ![micah](https://avatars.githubusercontent.com/u/71414271?v=4) | ![spencer](https://avatars.githubusercontent.com/u/122257729?v=4) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| [Jasper D](https://github.com/wluxie)  | [Cameron R](https://github.com/Supernova1114) | [Max R](https://github.com/max9001) | [Matin Q](https://github.com/MatinQurban) | [Sierra M](https://github.com/mcdipples) | [Micah H](https://github.com/micahh88) | [Spencer G](https://github.com/Gryuuum) |

## References
- Lost track, just google them or ask us if you need help ¯\\\_(ツ)_/¯