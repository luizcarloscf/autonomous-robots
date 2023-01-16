// Copyright 2022 Luiz Carlos Cosmi Filho and others.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <queue>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "turtlebot3_controller/action/task.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace std;

using Task = turtlebot3_controller::action::Task;
using GoalHandleTask = rclcpp_action::ServerGoalHandle<Task>;

/**
 * @brief ROS2 node responsible for controlling the position of a robot.
 *
 * It evaluates the current position through robot odometry and sends
 * angular/linear velocity commands to the robot to reach the target position.
 */
class TurtlebotController : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Turtlebot Controller object
   *
   * It instantiates a subscriber to receive robot odometry and a publisher to
   * send speed commands.
   */
  TurtlebotController();

 private:
  double kp_angular, kp_linear, tolerance, rate;
  bool odometry_init = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_twist_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp_action::Server<Task>::SharedPtr action_server_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;

  nav_msgs::msg::Odometry::SharedPtr odometry_;
  std::queue<std::shared_ptr<GoalHandleTask>> queue_;

  /**
   * @brief Callback executed to update parameters.
   *
   * @param parameters
   * @return rcl_interfaces::msg::SetParametersResult
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(
      const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Callback executed whenever a message arrives on topic /odom
   *
   * @param odometry Odometry message received.
   */
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr odometry);

  /**
   * @brief Callback used to receive cancel messages.
   *
   * @param goal_handle Point to goal handler.
   * @return rclcpp_action::CancelResponse
   */
  rclcpp_action::CancelResponse callback_cancel(
      const std::shared_ptr<GoalHandleTask> goal_handle);

  /**
   * @brief Callback used to receive goal and evaluates if it is an valid goal.
   *
   * @param uuid  goal unique identifier
   * @param goal Point to goal handler.
   * @return rclcpp_action::GoalResponse
   */
  rclcpp_action::GoalResponse callback_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const Task::Goal> goals);

  /**
   * @brief Callback used to accept goal messages.
   *
   * @param goal_handle Point to goal handler.
   */
  void callback_accepted(const std::shared_ptr<GoalHandleTask> goal_handle);

  /**
   * @brief Function responsible to execute all goals received. Deque and
   * execute,
   *
   */
  void execute();

  /**
   * @brief Compute the yaw angle from a given quartertion in the odometry.
   *
   * @param odom odometry message.
   * @return double yaw angle.
   */
  double euler_from_quartertion(const nav_msgs::msg::Odometry::SharedPtr odom);

  /**
   * @brief Compute the angular speed to be applied.
   *
   * @param error steering angle between current and goal position.
   * @return double angular speed to be applied.
   */
  double angular_speed(double error);

  /**
   * @brief Compute the linear speed to be applied.
   *
   * @param distance euclidean distance between current and goal position.
   * @return double linear speed to be applied.
   */
  double linear_speed(double distance);

  /**
   * @brief Calculates the angle between the current position and the goal
   * position. The first point is the current position and the second point is
   * the goal position.
   *
   * @param x1 first point's x value.
   * @param y1 first point's y value.
   * @param x2 second point's x value.
   * @param y2 second point's y value.
   * @return double
   */
  double steering_angle(double x1, double y1, double x2, double y2);

  /**
   * @brief Computes the euclidian distance between two points in a 2D plane.
   *
   * @param x1 first point's x value.
   * @param y1 first point's y value.
   * @param x2 second point's x value.
   * @param y2 secord point's y value.
   * @return double Euclidean distance between the two given points
   */
  double euclidian_distance(double x1, double y1, double x2, double y2);
};
