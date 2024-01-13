#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "crazyflie_msgs/action/trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


class TrajectoryActionClient : public rclcpp::Node
{
public:
  using Trajectory = crazyflie_msgs::action::Trajectory;
  using GoalHandleTrajectory = rclcpp_action::ClientGoalHandle<Trajectory>;

  explicit TrajectoryActionClient(const rclcpp::NodeOptions & options);

  void send_goal();

private:
  rclcpp_action::Client<Trajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleTrajectory::SharedPtr> future);

  void feedback_callback(
    GoalHandleTrajectory::SharedPtr,
    const std::shared_ptr<const Trajectory::Feedback> feedback);

  void result_callback(const GoalHandleTrajectory::WrappedResult & result);
};  // class TrajectoryActionClient


RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryActionClient)