import rclpy
import numpy as np
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rabbit_interfaces.msg import RabDict
from rclpy import qos


class DriveRabbit(Node):
    def __init__(self):
        super().__init__("drive_rabbit_node")
        self.dat = self.create_subscription(
            RabDict,
            "joystick_topic",
            self.sub_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.dat

        self.sent_drive = self.create_publisher(
            Twist, "drive_topic", qos_profile=qos.qos_profile_system_default
        )
        self.sent_drive_timer = self.create_timer(0.05, self.sent_drive_callback)

        self.axes = {}
        self.buttons = {}
        self.joyState = False

        self.distance = 0.0
        self.state_speed = 0
        self.preDriveMode = -1
        self.stateDriveMode = 0

        self.declare_parameters("", [("speed_motor", None), ("path_auto", None)])
        self.speed = (
            self.get_parameter("speed_motor").get_parameter_value().double_value
        ) / 255

    def sub_callback(self, msg):
        self.axes = dict(zip(msg.key_axes, msg.value_axes))
        self.buttons = dict(zip(msg.key_buttons, msg.value_buttons))
        self.joyState = msg.connection

    def sent_drive_callback(self):  # publisher drive topic
        msg = Twist()

        x = 0.0
        y = 0.0
        turn = 0.0
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        try:
            if self.preDriveMode != self.buttons["LS"]:
                self.preDriveMode = self.buttons["LS"]
                if self.buttons["LS"] == 1:
                    self.stateDriveMode += 1
            if self.stateDriveMode > 1:
                self.stateDriveMode = 0

            if self.state_speed == 0:
                if (self.axes["AX"] != 0) or (self.axes["AY"] != 0):
                    y = -1 * self.axes["AX"]
                    x = -1 * self.axes["AY"]
                    if self.stateDriveMode == 1:
                        x = np.interp(x, [-1, 1], [-1 * self.speed, self.speed])
                        y = np.interp(y, [-1, 1], [-1 * self.speed, self.speed])

                else:
                    if self.axes["LX"] != 0:
                        y = -1 * abs(self.axes["LX"]) / self.axes["LX"]
                    if self.axes["LY"] != 0:
                        x = -1 * abs(self.axes["LY"]) / self.axes["LY"]
                    if self.stateDriveMode == 1:
                        x = np.interp(x, [-1, 1], [-1 * self.speed, self.speed])
                        y = np.interp(y, [-1, 1], [-1 * self.speed, self.speed])

            y = y * -1
            if self.axes["RX"] != 0:
                turn = np.interp(-1 * abs(self.axes["RX"]) / self.axes["RX"], [-1, 1], [-1 * self.speed, self.speed])
            theta = math.atan2(y, x)
            power = math.hypot(x, y)
            sin = math.sin(theta - math.pi / 4)
            cos = math.cos(theta - math.pi / 4)
            Max = max(abs(sin), abs(cos))
            leftFront = power * cos / Max + turn
            rightFront = power * sin / Max + turn
            leftBack = power * sin / Max - turn
            rightBack = power * cos / Max - turn

            if (power + abs(turn)) > 1:
                leftFront /= power + abs(turn)
                rightFront /= power + abs(turn)
                leftBack /= power + abs(turn)
                rightBack /= power + abs(turn)

            msg.linear.x = float(round(leftFront * 255))
            msg.linear.y = float(round(rightFront * 255))
            msg.angular.x = float(round(leftBack * 255))
            msg.angular.y = float(round(rightBack * 255))

        except KeyError:
            pass

        if not self.joyState:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0

        self.get_logger().info(f"1")
        self.sent_drive.publish(msg)


def main():
    rclpy.init()

    sub = DriveRabbit()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
