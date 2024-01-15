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

import math
import time
import threading

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from turtlesim_interfaces.action import TurtleTask


class TurtlesimController(Node):
    """TurtlesimController is a node is ROS2 responsible for sending linear and angular velocity
    parameters to the turtle simulation node.
    """

    def parameter_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        """Callback function executed whenever it receives a message to handle parameters.

        Parameters
        ----------
        params: List[Parameter]
            Message with new parameters to update.

        Returns
        ------- 
        SetParametersResult
            Message with the result after executed parameter callback.
        """
        for param in parameters:
            if param.name == 'kp_angular':
                self.kp_angular = param.value
                self.get_logger().info("Updated parameter kp_angular=%.2f" % self.kp_angular)
            elif param.name == "kp_linear":
                self.kp_linear = param.value
                self.get_logger().info("Updated parameter kp_linear=%.2f" % self.kp_linear)
            elif param.name == "tolerance":
                self.delta = param.value
                self.get_logger().info("Updated parameter tolerance=%.2f" % self.delta)
            elif param.name == "sleep":
                self.sleep = param.value
                self.get_logger().info("Updated parameter sleep=%.2f" % self.sleep)
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def __init__(self, node_name: str = 'turtlesim_controller') -> None:
        """Constructs all the necessary attributes for the TurtlesimController object.

        Parameters
        ----------
            node_name: str = 'turtlesim_controller'
                ROS2 node name.
        """
        # initialize the node
        super(TurtlesimController,
              self).__init__(node_name=node_name,
                             automatically_declare_parameters_from_overrides=False)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        # Reentrant Callback Group allows the executor to schedule and execute the group's
        # callbacks in any way the it sees fit, without restrictions.
        callback_group = ReentrantCallbackGroup()
        # current position
        self.pose = Pose()
        # publisher to send commands to turtlesim_node
        self._publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10,
            callback_group=callback_group,
        )
        # subscribe to receive pose from turtlesim_node
        self._subscriber = self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10,
            callback_group=callback_group,
        )
        # action server
        self._action_server = ActionServer(
            node=self,
            action_type=TurtleTask,
            action_name='turtletask',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )
        # parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "kp_linear",
                    2.0,
                    ParameterDescriptor(description="Proportional constant to linear speed"),
                ),
                (
                    "kp_angular",
                    7.0,
                    ParameterDescriptor(description="Proportional constant to angular speed"),
                ),
                (
                    "tolerance",
                    0.1,
                    ParameterDescriptor(description="Acceptable final error"),
                ),
                (
                    "x_initial",
                    5.0,
                    ParameterDescriptor(description="x-axis initial position"),
                ),
                (
                    "y_initial",
                    5.0,
                    ParameterDescriptor(description="y-axis initial position"),
                ),
                (
                    "sleep",
                    0.01,
                    ParameterDescriptor(description="amount of time (s) sleep in action server"),
                ),
            ],
        )
        self.kp_linear = float(self.get_parameter("kp_linear").value)
        self.kp_angular = float(self.get_parameter("kp_angular").value)
        self.delta = float(self.get_parameter("tolerance").value)
        self.sleep = float(self.get_parameter("sleep").value)
        self.goal = Pose()
        self.goal.x = float(self.get_parameter("x_initial").value)
        self.goal.y = float(self.get_parameter("y_initial").value)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info("Init turtlesim_controller")

    def destroy(self) -> None:
        """Helper function to terminate action server and node.
        """
        self._action_server.destroy()
        super(TurtlesimController, self).destroy_node()

    def goal_callback(self, goal_request: TurtleTask.Goal):
        """Callback function for handling new goal requests.

        Parameters
        ----------
        params: TurtleTask.Goal
            Message with new goal request.

        Returns
        -------
        GoalResponse.ACCEPT
            Always returns accept if received action goal request.
        """
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """Callback function for handling newly accepted goals.

        Parameters
        ----------
        goal_handle: ServerGoalHandle
            Passes an instance of `ServerGoalHandle` as an argument.
        """
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse.ACCEPT:
        """Callback function for handling cancel requests.

        Parameters
        ----------
        goal_handle: ServerGoalHandle
            Passes an instance of `ServerGoalHandle` as an argument.

        Returns
        -------
        CancelResponse.ACCEPT
            Always return an message saying that received cancel message.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> TurtleTask.Result:
        """Callback function for processing accepted goals. This is called if when
        `ServerGoalHandle.execute()` is called for a goal handle that is being tracked by
        this action server.

        Parameters
        ----------
        goal_handle: ServerGoalHandle
            Passes an instance of `ServerGoalHandle` as an argument.

        Returns
        -------
        TurtleTask.Result
            Result position after the system converges.
        """
        self.get_logger().info('Sending turtle...')
        self.goal.x = goal_handle.request.x
        self.goal.y = goal_handle.request.y

        current = self.pose
        distance = self.euclidean_distance(current=current, target=self.goal)
        while distance >= self.delta:

            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                response = TurtleTask.Result()
                response.x = current.x
                response.y = current.y
                return response

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                response = TurtleTask.Result()
                response.x = current.x
                response.y = current.y
                return response

            msg = Twist()
            # linear msg components
            msg.linear.x = self.linear_speed(distance=distance)
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            # angular msg components
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.angular_speed(current=current)
            # log current error
            self.get_logger().info('Error %.2f' % (distance))
            # send feedback
            feedback_msg = TurtleTask.Feedback()
            feedback_msg.x = self.pose.x
            feedback_msg.y = self.pose.y
            goal_handle.publish_feedback(feedback_msg)
            # send message
            self._publisher.publish(msg)
            time.sleep(self.sleep)
            # get distance
            current = self.pose
            distance = self.euclidean_distance(current=current, target=self.goal)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._publisher.publish(msg)
        goal_handle.succeed()
        self.get_logger().info("Reached goal! :)")

        response = TurtleTask.Result()
        response.x = self.pose.x
        response.y = self.pose.y
        return response

    def pose_callback(self, pose: Pose) -> None:
        """Callback function executed whenever it receives a message from topic subscribed. that
        is, updates the robot's position.

        Parameters
        ----------
        pose: Pose
            current robot's position.
        """
        self.pose = pose

    def linear_speed(self, distance) -> float:
        """Compute the linear speed to be applied.

        Parameters
        ----------
        distance: float
            euclidean distance between current and target position.

        Returns
        -------
        float
            linear speed to be applied.
        """
        return self.kp_linear * math.tanh(distance)

    def steering_angle(self, current: Pose) -> float:
        """Calculates the angle between the current position and the target position.

        Parameters
        ----------
        current: Pose
            robot's current position.

        Returns
        -------
        float
            returns the angle between current position and the target position measured in radians.
        """
        return math.atan2(
            self.goal.y - self.pose.y,
            self.goal.x - self.pose.x,
        )

    def angular_speed(self, current) -> float:
        """Compute the angular speed to be applied.

        Parameters
        ----------
        current: Pose
            robot's current position.

        Returns
        -------
        float
            angular speed to be applied.
        """
        error = self.steering_angle(current=current) - current.theta
        if error > math.pi:
            error -= 2 * math.pi
        if error < -math.pi:
            error += 2 * math.pi
        return self.kp_angular * (error)

    @staticmethod
    def euclidean_distance(current: Pose, target: Pose) -> float:
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


def main(*args, **kwargs):
    # init ROS2 client communications
    rclpy.init(*args, **kwargs)
    # using MultiThreadExecutor to be able to process multiple callback, because the action is
    # a long-running callback.
    executor = MultiThreadedExecutor()
    # creates a node
    node = TurtlesimController()
    try:
        # keeps node alive
        rclpy.spin(node=node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
