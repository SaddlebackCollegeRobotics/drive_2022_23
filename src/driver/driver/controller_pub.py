import rclpy
from rclpy.node import Node
from . import gamepad_input as gmi
from .buttons import Buttons

from std_msgs.msg import Float64MultiArray



AXIS_DEADZONE = 0.3 # Deadzone is 0 to 1 | Note: axis value will be 0 until you move past the deadzone

b = Buttons()
# buttonDownEvents = [
#     b.north, b.west, b.south, b.east,
#     b.share, b.options, b.home,                       
#     b.l1, b.r1, b.l3, b.r3]
# buttonUpEvents = [
#     b.northUp, b.westUp, b.southUp, b.eastUp, 
#     b.shareUp, b.optionsUp, b.homeUp, 
#     b.l1Up, b.r1Up, b.l3Up, b.r3Up]
# hatEvents = [b.hatNorth, b.hatWest, b.hatSouth, b.hatEast, b.hatCentered]
connectionEvents = [b.onGamepadConnect, b.onGamepadDisconnect]


class ControllerPub(Node):
    def __init__(self):
        super().__init__('controller_pub')

        self.publisher_ = self.create_publisher(Float64MultiArray, 'controls', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # gmi.run_event_loop(buttonDownEvents, buttonUpEvents, hatEvents, connectionEvents)   # Async loop to handle gamepad button events
        gmi.run_event_loop(None, None, None, connectionEvents)   # Async loop to handle gamepad button events


    def timer_callback(self):
        gp = gmi.getGamepad(0)

        (ls_x, ls_y) = gmi.getLeftStick(gp, AXIS_DEADZONE)  # Get left stick
        (rs_x, rs_y) = gmi.getRightStick(gp, AXIS_DEADZONE) # Get right stick
        (l2, r2) = gmi.getTriggers(gp, AXIS_DEADZONE)       # Get triggers

        msg = Float64MultiArray()
        msg.data = [float(ls_x), float(ls_y), float(rs_x), float(rs_y), float(l2), float(r2)]

        self.publisher_.publish(msg)
        # self.get_logger().info('SENDED: "%s"' % msg.data)
        print('😤😤 SENDING [LS: (%.2f, %.2f) | RS: (%.2f, %.2f) | LT: %.2f | RT: %.2f] 😤😤' % (ls_x, ls_y, rs_x, rs_y, l2, r2))


def main(args=None):
    rclpy.init(args=args)

    controller_pub = ControllerPub()

    rclpy.spin(controller_pub)

    controller_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()