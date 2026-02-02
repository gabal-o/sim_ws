from sshkeyboard import listen_keyboard
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "inputs", 10)
        listen_keyboard(on_press=self.press)

    def press(self, key):
        msg=String()
        msg.data=key
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: key={msg.data}")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
