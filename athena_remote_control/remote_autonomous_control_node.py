import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


class RemoteControl(LifecycleNode):

    def __init__(self):
        super().__init__("remote_control")

        self.prev_buttons = None
        self.emergency_pub = None
        self.autonomy_toggle_pub = None
        self.remot_drive_controller_sub = None

    def on_configure(self, state: State):
        self.get_logger().info("Configuring...")

        controller_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.emergency_pub = self.create_lifecycle_publisher(
            Bool, "/emergency_stop", 10
        )

        self.autonomy_toggle_pub = self.create_lifecycle_publisher(
            Bool, "/autonomy_toggle", 10
        )

        self.remot_drive_controller_sub = self.create_subscription(
            Joy, "/joy", self.remote_drive_controller_callback, controller_qos
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("Activating...")
        self.emergency_pub.activate()
        self.autonomy_toggle_pub.activate()

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info("Deactivating...")
        self.emergency_pub.deactivate()
        self.autonomy_toggle_pub.deactivate()
        self.prev_buttons = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info("Cleaning up...")

        self.destroy_publisher(self.emergency_pub)
        self.destroy_publisher(self.autonomy_toggle_pub)
        self.destroy_subscription(self.remot_drive_controller_sub)

        return TransitionCallbackReturn.SUCCESS

    def remote_drive_controller_callback(self, joy: Joy):
        if self.prev_buttons is None:
            self.prev_buttons = joy.buttons
            return

        # X Button (Index 0)
        if self.prev_buttons[0] == 0 and joy.buttons[0] == 1:
            msg = Bool()
            msg.data = True
            if self.autonomy_toggle_pub.is_activated:
                self.autonomy_toggle_pub.publish(msg)
            self.get_logger().info("Autonomous mode activated")

        # O Button (Index 1)
        if self.prev_buttons[1] == 0 and joy.buttons[1] == 1:
            msg = Bool()
            msg.data = True
            if self.emergency_pub.is_activated:
                self.emergency_pub.publish(msg)
            self.get_logger().warn("EMERGENCY STOP")

        self.prev_buttons = joy.buttons


def main():
    rclpy.init()
    node = RemoteControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
