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

        self.current_speed = 0.0

        controller_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.remot_controller_sub = self.create_subscription(
            Joy, "/joy", self.remote_controller_callback, controller_qos
        )

        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped, "/ackermann_cmd", controller_qos
        )

    def remote_controller_callback(self, joy: Joy):

        steering = self.steering_mapping(joy.axes[0])

        target = self.speed_mapping(joy.axes[5])

        self.current_speed = self.ramp(
            target, self.current_speed, accel_limit=0.05, decel_limit=0.05
        )

        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()

        ackermann_msg.drive.speed = self.current_speed
        ackermann_msg.drive.steering_angle = steering

        self.ackermann_pub.publish(ackermann_msg)

    def ramp(self, target, current, accel_limit, decel_limit):
        if target > current:
            return min(target, current + accel_limit)
        else:
            return max(target, current - decel_limit)

    def steering_mapping(self, joy_steering_value):
        return 0.3 * joy_steering_value

    def speed_mapping(self, joy_speed_value):
        return -3 / 2 * joy_speed_value + 3 / 2


def main():
    rclpy.init()

    node = RemoteControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
