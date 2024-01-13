#include <functional>
#include <memory>
#include <thread>

#include "crazyflie_msgs/action/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action/visibility_control.h"


class TrajectoryActionServer : public rclcpp::Node
{
public:
  using Trajectory = crazyflie_msgs::action::Trajectory;
  using GoalHandleTrajectory = rclcpp_action::ServerGoalHandle<Trajectory>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit TrajectoryActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trajectory_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Trajectory>(
      this,
      "trajectory",
      std::bind(&TrajectoryActionServer::handle_goal, this, _1, _2),
      std::bind(&TrajectoryActionServer::handle_cancel, this, _1),
      std::bind(&TrajectoryActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Trajectory>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Trajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received trajectory generation request with %d target waypoints.", goal->n_target_waypoints);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TrajectoryActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Trajectory::Feedback>();
    auto & feedback_msg = feedback->message;

    // first feedback
    feedback_msg = "Started trajectory generation.";
    goal_handle->publish_feedback(feedback);

    const auto target_waypoints = goal->target_waypoints;
    const auto target_timestamps = goal->target_timestamps;
    const auto discretization_count = goal->discretization_count;

    auto result = std::make_shared<Trajectory::Result>();

    if (goal_handle->is_canceling()) {
        //result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      // result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class TrajectoryActionServer


RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryActionServer)