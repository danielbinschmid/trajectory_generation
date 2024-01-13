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
  explicit TrajectoryActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Trajectory>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Trajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrajectory> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleTrajectory> goal_handle);
  void execute(const std::shared_ptr<GoalHandleTrajectory> goal_handle);
}; 

RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryActionServer)