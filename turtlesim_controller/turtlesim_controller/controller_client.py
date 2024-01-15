# Copyright 2022 Luiz Carlos Cosmi Filho and others.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

import rclpy

from action_msgs.msg import GoalStatus
from turtlesim_interfaces.action import TurtleTask

from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future


class TurtleTaskActionClient(Node):

    def __init__(self, node_name: str = 'turtletask_action_client') -> None:
        """
        Initialize the TurtleTaskActionClient object.

        :param node_name: str, optional
            ROS2 node name. Default is 'turtletask_action_client'.
        """
        super().__init__(node_name=node_name)
        self._action_client = ActionClient(self, TurtleTask, 'turtletask')

    def goal_response_callback(self, future: Future) -> None:
        """
        Handle responses from the action server.

        :param future: rclpy.task.Future
            Represents the outcome of a task in the future.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
        else:
            self.get_logger().info('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback: TurtleTask.Feedback) -> None:
        """
        Handle feedback messages from the action server.

        :param feedback: TurtleTask.Feedback
            Represents the current outcome of a task.
        """
        self.get_logger().info(
            'Received feedback: %.2f, %.2f' % (
                feedback.feedback.x,
                feedback.feedback.y,
            ),
        )

    def get_result_callback(self, future: Future) -> None:
        """
        Handle result messages from the action server.

        Shut down the node after the goal succeeded or not.

        :param future: rclpy.task.Future
            Represents the outcome of a task in the future.
        """
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

    def send_goal(self, x: float, y: float) -> None:
        """
        Send a message to an action server and handle its responses.

        :param x: float
            Goal position in x-axis.
        :param y: float
            Goal position in y-axis.
        """
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        goal_msg = TurtleTask.Goal()
        goal_msg.x = x
        goal_msg.y = y
        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description='Move turtle to goal.')
    parser.add_argument('-x',
                        action='store',
                        dest='x',
                        type=float,
                        default=5.0,
                        required=True,
                        help='Goal position in the x-axis')
    parser.add_argument('-y',
                        action='store',
                        dest='y',
                        type=float,
                        default=5.0,
                        required=True,
                        help='Goal position in the y-axis')
    # getting the arguments
    args = parser.parse_args()
    # init ROS2 client communications
    rclpy.init(args=None)
    # init node
    action_client = TurtleTaskActionClient()
    # fire message to action server
    action_client.send_goal(x=args.x, y=args.y)
    try:
        # keeps node alive until received goal response
        rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        action_client.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
