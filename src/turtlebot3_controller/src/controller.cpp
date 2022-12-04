#include "turtlebot3_controller/controller.hpp"

using Task = turtlebot3_controller::action::Task;
using GoalHandleTask = rclcpp_action::ServerGoalHandle<Task>;

TurtlebotController::TurtlebotController() : Node("turtlebot_controller") {
  // setting up parameters to configure node
  this->declare_parameter("kp_angular", 0.5);
  this->declare_parameter("kp_linear", 0.5);
  this->declare_parameter("tolerance", 0.1);
  this->declare_parameter("rate", 0.5);

  this->kp_angular = this->get_parameter("kp_angular").as_double();
  this->kp_linear = this->get_parameter("kp_linear").as_double();
  this->tolerance = this->get_parameter("tolerance").as_double();
  this->rate = this->get_parameter("rate").as_double();

  // subscriber to receibe robots odometry
  this->subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TurtlebotController::callback_odometry, this,
                std::placeholders::_1));
  // publisher to send vel commands
  this->publisher_twist_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // action server to execute tasks
  this->action_server_ = rclcpp_action::create_server<Task>(
      this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_waitables_interface(),
      "task",
      std::bind(&TurtlebotController::callback_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&TurtlebotController::callback_cancel, this,
                std::placeholders::_1),
      std::bind(&TurtlebotController::callback_accepted, this,
                std::placeholders::_1));
  // add parameter handler
  this->parameters_ = this->add_on_set_parameters_callback(std::bind(
      &TurtlebotController::parameters_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Node started...");
}

rcl_interfaces::msg::SetParametersResult
TurtlebotController::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto& param : parameters) {
    if (param.get_name() == "kp_angular") {
      this->kp_angular = param.as_double();
    } else if (param.get_name() == "kp_linear") {
      this->kp_linear = param.as_double();
    } else if (param.get_name() == "tolerance") {
      this->tolerance = param.as_double();
    } else if (param.get_name() == "rate") {
      this->rate = param.as_double();
    }
  }
  return result;
}

void TurtlebotController::callback_odometry(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  this->odometry_ = odom;
  this->odometry_init = true;
}

rclcpp_action::CancelResponse TurtlebotController::callback_cancel(
    const std::shared_ptr<GoalHandleTask> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Got request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse TurtlebotController::callback_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const Task::Goal> goals) {
  RCLCPP_INFO(this->get_logger(), "Got goal request");
  (void)uuid;
  if (goals->x.size() != goals->y.size()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (long unsigned int i = 0; i < goals->x.size(); i++) {
    if ((goals->x[i] > 10.0) || (goals->y[i] > 10.0) || (goals->x[i] < -10.0) ||
        (goals->y[i] < -10.0) || (!this->odometry_init)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void TurtlebotController::callback_accepted(
    const std::shared_ptr<GoalHandleTask> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Accepted goal");
  if (this->queue_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Started thread");
    this->queue_.push(goal_handle);
    std::thread{std::bind(&TurtlebotController::execute, this)}.detach();
  } else {
    RCLCPP_INFO(this->get_logger(), "Add to queue");
    this->queue_.push(goal_handle);
  }
}

void TurtlebotController::execute() {
  auto current = this->odometry_;
  auto result = std::make_shared<Task::Result>();
  auto feedback = std::make_shared<Task::Feedback>();
  auto goal_handle = this->queue_.front();

  while (!this->queue_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(this->rate);
    auto goal_handle = this->queue_.front();
    const auto goals = goal_handle->get_goal();

    for (long unsigned int i = 0; i < goals->x.size(); i++) {
      current = this->odometry_;
      auto distance = this->euclidian_distance(current->pose.pose.position.x,
                                               current->pose.pose.position.y,
                                               goals->x[i], goals->y[i]);
      while (distance > this->tolerance && rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
          auto message = geometry_msgs::msg::Twist();
          message.linear.x = 0.0;
          message.linear.y = 0.0;
          message.linear.z = 0.0;
          message.angular.x = 0.0;
          message.angular.y = 0.0;
          message.angular.z = 0.0;
          this->publisher_twist_->publish(message);
          result->x = current->pose.pose.position.x;
          result->y = current->pose.pose.position.y;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal Canceled");
          return;
        }
        double theta = this->euler_from_quartertion(current);
        double alpha = this->steering_angle(current->pose.pose.position.x,
                                            current->pose.pose.position.y,
                                            goals->x[i], goals->y[i]);

        double error = alpha - theta;
        if (error > M_PI)
          error -= 2 * M_PI;
        if (error < -M_PI)
          error += 2 * M_PI;

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = this->linear_speed(distance);
        message.linear.y = 0.0;
        message.linear.z = 0.0;

        message.angular.x = 0.0;
        message.angular.y = 0.0;
        message.angular.z = this->angular_speed(error);

        feedback->x = current->pose.pose.position.x;
        feedback->y = current->pose.pose.position.y;

        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish Feedback");

        this->publisher_twist_->publish(message);
        loop_rate.sleep();

        current = this->odometry_;
        distance = this->euclidian_distance(current->pose.pose.position.x,
                                            current->pose.pose.position.y,
                                            goals->x[i], goals->y[i]);
      }
    }
    this->queue_.pop();
    result->x = current->pose.pose.position.x;
    result->y = current->pose.pose.position.y;
    goal_handle->succeed(result);
  }
  if (rclcpp::ok()) {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.linear.y = 0.0;
    message.linear.z = 0.0;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = 0.0;
    this->publisher_twist_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }
}

double TurtlebotController::euler_from_quartertion(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  double x = odom->pose.pose.orientation.x;
  double y = odom->pose.pose.orientation.y;
  double z = odom->pose.pose.orientation.z;
  double w = odom->pose.pose.orientation.w;
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

double TurtlebotController::angular_speed(double error) {
  return this->kp_angular * error;
}

double TurtlebotController::linear_speed(double distance) {
  return this->kp_linear * std::tanh(distance);
}

double TurtlebotController::steering_angle(double x1,
                                           double y1,
                                           double x2,
                                           double y2) {
  return std::atan2((y2 - y1), (x2 - x1));
}

double TurtlebotController::euclidian_distance(double x1,
                                               double y1,
                                               double x2,
                                               double y2) {
  return std::sqrt(std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2));
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlebotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}