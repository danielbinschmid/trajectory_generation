#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "crazyflie_msgs/action/trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "traj_action_client.hpp"

TrajectoryActionClient::TrajectoryActionClient(const rclcpp::NodeOptions & options)
  : Node("trajectory_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Trajectory>(
      this,
      "trajectory");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TrajectoryActionClient::send_goal, this));
  }

void TrajectoryActionClient::send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Trajectory::Goal();

    // define goal
    goal_msg.target_waypoints = {0, 0, 0, 16, 16, 16};
    goal_msg.target_timestamps = {0, 10};
    goal_msg.discretization_count = 10;
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Trajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TrajectoryActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TrajectoryActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TrajectoryActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }


  void TrajectoryActionClient::goal_response_callback(std::shared_future<GoalHandleTrajectory::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void TrajectoryActionClient::feedback_callback(
    GoalHandleTrajectory::SharedPtr,
    const std::shared_ptr<const Trajectory::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Feedback received: %s", feedback->message.c_str());
  }

  void TrajectoryActionClient::result_callback(const GoalHandleTrajectory::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }


