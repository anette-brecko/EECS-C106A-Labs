import rclpy
from rclpy.node import Node
import sys

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class Turtle1PatrolServer(Node):

    def __init__(self):

        super().__init__('turtle1_patrol_server')
        self._srv = self.create_service(Patrol, '/turtle1/patrol', self.patrol_callback)

        # List that keeps track of all turtles
        self.turts = []

        # Current commanded speeds (what timer publishes)
        self._lin = 0.0
        self._ang = 0.0

        # Timer: publish current speeds at 10 Hz
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)

        self.get_logger().info('Turtle1PatrolServer ready (continuous publish mode).')

    # -------------------------------------------------------
    # Timer publishes current Twist
    # -------------------------------------------------------
    def _publish_current_cmd(self):
        for i in self.turts:
            msg = Twist()
            msg.linear.x = i["vel"]
            msg.angular.z = i["omega"]
            curr_pub = i["publisher"]
            curr_pub.publish(msg)

    # -------------------------------------------------------
    # Service callback: update speeds
    # -------------------------------------------------------
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        self.get_logger().info(
            f"Patrol request: turtle_name={request.turtle_name}, vel={request.vel:.2f}, omega={request.omega:.2f}, x={request.x:.2f}, y={request.y:.2f}, theta={request.theta:.2f}"
        )

        # Checks if this turtle already exists
        turt_exists = False
        for i in self.turts:
            if i["name"] == request.turtle_name:
                turt_exists = True 
                break

        # Creating a new publisher and adding the turtle to turts[] if it does not exist
        if turt_exists == False:
            curr_topic = '/' + request.turtle_name + '/cmd_vel'
            cmd_pub = self.create_publisher(Twist, curr_topic, 10)
            this_turt = {"name": request.turtle_name, "vel": request.vel, "omega": request.omega, "publisher": cmd_pub}
            self.turts.append(this_turt)

        # If turtle already exists, change vel and omega
        elif turt_exists == True:
            for i in self.turts:
                if i["name"] == request.turtle_name:
                    i["vel"] = request.vel
                    i["omega"] = request.omega

        curr_tel = '/' + request.turtle_name + '/teleport_absolute'
        self._client = self.create_client(TeleportAbsolute, curr_tel)
        req = TeleportAbsolute.Request()
        req.x = request.x
        req.y = request.y
        req.theta = request.theta
        self._future = self._client.call_async(req)

        # Update the speeds that the timer publishes
        self._lin = float(request.vel)
        self._ang = float(request.omega)

        # Prepare response Twist reflecting current command
        cmd = Twist()
        cmd.linear.x = self._lin
        cmd.angular.z = self._ang
        response.cmd = cmd

        self.get_logger().info(
            f"Streaming cmd_vel: lin.x={self._lin:.2f}, ang.z={self._ang:.2f} (10 Hz)"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Turtle1PatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
