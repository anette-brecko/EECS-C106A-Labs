import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        curr_turt = str(sys.argv[1])
        topic = '/' + curr_turt + '/cmd_vel'
        self.publisher_ = self.create_publisher(Twist, topic, 10)
        

    def callback(self):

        msg = Twist()
        key = input()

        #upwards motion
        if (key == 'w' or key == 'W'):
            msg.linear.x = 2.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        #downwards motion
        if (key == 's' or key == 'S'):
            msg.linear.x = -2.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        #counterclockwise motion
        if (key == 'a' or key == 'A'):
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 2.0

        #clockwise motion
        if (key == 'd' or key == 'D'):
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = -2.0
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    print("Control the turtle using WASD (Press <Enter> after every keystroke)")
    while rclpy.ok():
        minimal_publisher.callback()

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()