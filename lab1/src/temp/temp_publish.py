
import rclpy
from rclpy.node import Node

from my_chatter_msgs.msg import TimestampString

class MinimalPublisher(Node):

    # Here, we define the constructor
    def __init__(self):
        # We call the Node class's constructor and call it "my_publisher"
        super().__init__('minimal_publisher')
        
        self.publisher_ = self.create_publisher(String, 'chatter_talk', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.create_message)
        self.i = 0
        

    def create_message(self):
        msg = TimestampString()
        msg.message = input("Please enter a line of text and press <Enter>")
        msg.timestamp = Node.get_clock.now().nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.message)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    minimal_publisher = MinimalPublisher()
    # Spin the node so its callbacks are called
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

