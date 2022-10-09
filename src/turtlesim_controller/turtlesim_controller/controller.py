import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_interfaces.srv import Trigger


class TurtlesimController(Node):
    """TurtlesimController is a node is ROS2 responsible for sending linear and angular velocity
    parameters to the turtle simulation node.
    """

    def __init__(self,
                 node_name: str = 'turtlesim_controller',
                 x_initial: float = 6.0,
                 y_initial: float = 5.0,
                 tolerance_initial: float = 0.1):
        """Constructs all the necessary attributes for the TurtlesimController object.

        Parameters
        ----------
        node_name: str
            identifies to node.
        x_initial:
            x-axis initial position.
        y_initial:
            y-axis initial position.
        tolerance_initial:
            initial tolerance.
        """
        # initialize the node
        super(TurtlesimController, self).__init__(node_name=node_name)
        # random initial objective
        self.goal = Pose()
        self.goal.x = x_initial
        self.goal.y = y_initial
        # current position
        self.pose = Pose()
        # tolerance
        self.delta = tolerance_initial
        # subscribe to receive pose from turtlesim_node
        self.pose_subscriber = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10,
        )
        # publisher to send commands to turtlesim_node
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10,
        )
        # create a timer with callback to calculate the speed and send to the node
        self.create_timer(
            timer_period_sec=0.1,
            callback=self.send_velocity_command,
        )
        # create a service to receive requests to modify the turtlesim_node's target position
        self.service = self.create_service(Trigger, 'move_to', self.goal_callback)
        self.get_logger().info("Init turtlesim_controller")

    def goal_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Callback function executed whenever it receives a message for the move_to service.

        Parameters
        ----------
        request: Trigger.Request
            received payload with new target position.
        response: Trigger.Response
            response informing if it was possible to store the received payload.
        """
        try:
            self.goal = Pose()
            self.goal.x = request.x
            self.goal.y = request.y
            self.delta = request.delta
            self.get_logger().info(
                'Incoming request\nx: %.2f y: %.2f delta: %.2f' % (
                    request.x,
                    request.y,
                    request.delta,
                ),
            )
            response.success = True
            response.message = "Received request"
        except:
            response.success = False
            response.message = "Something went wrong. Please, try again."
        return response

    def pose_callback(self, pose: Pose):
        """Callback function executed whenever it receives a message from topic subscribed. that
        is, updates the robot's position.

        Parameters
        ----------
        pose: Pose
            current robot's position.
        """
        self.pose = pose

    def linear_speed(self, distance, kp=1.5):
        """Compute the linear speed to be applied.

        Parameters
        ----------
        distance: float
            euclidean distance between current and target position.
        kp: float
            proportional gain.

        Returns
        -------
        float
            linear speed to be applied.
        """
        return kp * distance

    def steering_angle(self):
        """Calculates the angle between the current position and the target position.

        Returns
        -------
        float
            returns the angle between current position and the target position measured in radians.
        """
        return math.atan2(
            self.goal.y - self.pose.y,
            self.goal.x - self.pose.x,
        )

    def angular_speed(self, kp=6):
        """Compute the angular speed to be applied.

        Parameters
        ----------
        kp: float
            proportional gain.

        Returns
        -------
        float
            angular speed to be applied.
        """
        return kp * (self.steering_angle() - self.pose.theta)

    @staticmethod
    def euclidean_distance(current: Pose, target: Pose):
        """Static method used to compute the euclidean distance between to Pose.

        Parameters
        ----------
        current: Pose
            robot's current position.
        target: Pose
            robot's target position.

        Returns
        -------
        float
            distance between the two positions.
        """
        return math.sqrt((target.x - current.x)**2 + (target.y - current.y)**2)

    def send_velocity_command(self):
        """Callback function executated by timer. Responsible for assessing the robot's situation
        and sending messages of appropriate angular and linear speed.
        """
        msg = Twist()
        # linear msg components
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        # angular msg components
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # compute euclidean distance
        distance = self.euclidean_distance(current=self.pose, target=self.goal)
        # log current error
        self.get_logger().info('Error %.2f' % (distance))
        # if error to be big, apply some linear ang angular speed
        if distance >= self.delta:
            msg.linear.x = self.linear_speed(distance=distance)
            msg.angular.z = self.angular_speed()
        # send message
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    # init ROS2 client communications
    rclpy.init(args=args)
    # creates a node
    node = TurtlesimController()
    # keeps node alive
    rclpy.spin(node=node)
    # destroys all nodes and ROS2 communications
    rclpy.shutdown()


if __name__ == "__main__":
    main()
