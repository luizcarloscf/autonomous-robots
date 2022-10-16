import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from action_msgs.msg import GoalStatus

from turtlesim_interfaces.action import TurtleTask


class TurtleTaskActionClient(Node):

    def __init__(self):
        super().__init__('turtletask_action_client')
        self._action_client = ActionClient(self, TurtleTask, 'turtletask')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(
            'Received feedback: %.2f, %.2f' % (
                feedback.feedback.x,
                feedback.feedback.y,
            ),
        )

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Goal succeeded! Result: %.2f, %.2f' % (
                    result.x,
                    result.y,
                ),
            )
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self, x: float, y: float):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = TurtleTask.Goal()
        goal_msg.x = x
        goal_msg.y = y

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    parser = argparse.ArgumentParser(description='Move turtle to goal.')
    parser.add_argument('-x',
                        action='store',
                        dest='x',
                        type=float,
                        default=5.0,
                        required=True,
                        help='Coordinate on the x-axis')
    parser.add_argument('-y',
                        action='store',
                        dest='y',
                        type=float,
                        default=5.0,
                        required=True,
                        help='Coordinate on the y-axis')
    # getting the arguments
    args = parser.parse_args()
    # init ROS2 client communications
    rclpy.init(args=None)

    action_client = TurtleTaskActionClient()
    action_client.send_goal(x=args.x, y=args.y)

    try:
        # keeps node alive
        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        action_client.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
