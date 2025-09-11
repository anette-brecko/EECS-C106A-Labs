import rclpy
from rclpy.node import Node

from my_chatter_msgs.msg import TimestampString


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            TimestampString,
            'chatter_talk',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds
        self.get_logger().info(f"Message: {msg.message},  Sent at: {msg.timestamp}, Received at: {current_time}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
