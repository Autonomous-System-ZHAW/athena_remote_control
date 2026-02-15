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

        self.button_actions = {
            11: (self.up_toggle_button, "Upward selection", False),
            12: (self.down_toggle_button, "Downward selection", False),
            3: (self.autonomy_toggle_pub, "Autonomous mode activated", False),
            0: (self.confirm_toggle_pub, "Confirm selection ...", False),
            2: (
                self.mission_selection_toggle_pub,
                "Mission selection mode activated",
                False,
            ),
            1: (self.emergency_pub, "EMERGENCY STOP", True),  # True = warn log
        }

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

        self.confirm_toggle_pub = self.create_publisher(Bool, "/confirm_toggle", 10)

        self.mission_selection_toggle_pub = self.create_publisher(
            Bool, "/mission_toggle", 10
        )

        self.up_toggle_button = self.create_publisher(Bool, "/up_toggle", 10)

        self.down_toggle_button = self.create_publisher(Bool, "/down_toggle", 10)

        self.manuel_toggle = self.create_publisher(Bool, "/manuel_toggle", 10)

    def remote_drive_controller_callback(self, joy: Joy):
        if self.prev_buttons is None:
            self.prev_buttons = joy.buttons
            return

        # Standard Single-Button Actions
        for idx, (publisher, message, is_warn) in self.button_actions.items():
            if self.prev_buttons[idx] == 0 and joy.buttons[idx] == 1:
                publisher.publish(Bool(data=True))
                if is_warn:
                    self.get_logger().warn(message)
                else:
                    self.get_logger().info(message)

        # Special case: R1 + L1 together
        if (
            self.prev_buttons[9] == 0
            and self.prev_buttons[10] == 0
            and joy.buttons[9] == 1
            and joy.buttons[10] == 1
        ):
            self.manuel_toggle.publish(Bool(data=True))
            self.get_logger().info("Manual mode activated")

        self.prev_buttons = joy.buttons


def main():
    rclpy.init()

    node = RemoteControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
