import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


class RemoteControl(Node):

    def __init__(self):
        super().__init__("remote_control")

        self.prev_buttons = None

        controller_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.remot_drive_controller_sub = self.create_subscription(
            Joy, "/joy", self.remote_drive_controller_callback, controller_qos
        )

        self.emergency_pub = self.create_publisher(Bool, "/emergency_stop", 10)

        self.autonomy_toggle_pub = self.create_publisher(Bool, "/autonomy_toggle", 10)

    def remote_drive_controller_callback(self, joy: Joy):
        if self.prev_buttons is None:
            self.prev_buttons = joy.buttons
            return

        # X Button (Index 0)
        if self.prev_buttons[0] == 0 and joy.buttons[0] == 1:
            msg = Bool()
            msg.data = True
            self.autonomy_toggle_pub.publish(msg)
            self.get_logger().info("Autonomous mode activated")

        # O Button (Index 1)
        if self.prev_buttons[1] == 0 and joy.buttons[1] == 1:
            msg = Bool()
            msg.data = True
            self.emergency_pub.publish(msg)
            self.get_logger().warn("EMERGENCY STOP")

        self.prev_buttons = joy.buttons


def main():
    rclpy.init()

    node = RemoteControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
