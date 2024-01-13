#include <functional>
#include <memory>
#include <thread>
#include <stdexcept>
#include "crazyflie_msgs/action/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <Eigen/Eigen>

#include "action/visibility_control.h"
#include "traj_action_server.hpp"

TrajectoryActionServer::TrajectoryActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("trajectory_action_server", options)
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Trajectory>(
        this,
        "trajectory",
        std::bind(&TrajectoryActionServer::handle_goal, this, _1, _2),
        std::bind(&TrajectoryActionServer::handle_cancel, this, _1),
        std::bind(&TrajectoryActionServer::handle_accepted, this, _1));
    
    this->opt_type = traj::OptType::SNAP;
}


rclcpp_action::GoalResponse TrajectoryActionServer::handle_goal(
const rclcpp_action::GoalUUID & uuid,
std::shared_ptr<const Trajectory::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received trajectory generation request with %d target waypoints.", goal->n_target_waypoints);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryActionServer::handle_cancel(
const std::shared_ptr<GoalHandleTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryActionServer::handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TrajectoryActionServer::execute, this, _1), goal_handle}.detach();
}

void TrajectoryActionServer::execute(const std::shared_ptr<GoalHandleTrajectory> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Trajectory::Feedback>();
    auto & feedback_msg = feedback->message;

    // first feedback
    feedback_msg = "Started trajectory generation.";
    goal_handle->publish_feedback(feedback);

    const std::vector<float> target_waypoints = goal->target_waypoints;
    const auto target_timestamps = goal->target_timestamps;
    const auto discretization_count = goal->discretization_count;
    const auto n_target_waypoints = goal->n_target_waypoints;

    // convert target waypoints to Eigen matrix
    std::vector<double> target_waypoints_d(target_waypoints.begin(), target_waypoints.end());
    if (target_waypoints_d.size() % 3 != 0) { throw std::runtime_error("Target waypoints vector size is not divisible by 3"); }
    Eigen::MatrixXd mat = Eigen::Map<Eigen::MatrixXd>(target_waypoints_d.data(), 3, target_waypoints_d.size() / 3);
    
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


