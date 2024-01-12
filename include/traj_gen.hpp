#pragma once


#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>


using namespace Eigen;

/**
 * Whether to use JERK or SNAP optimization
 **/
enum OptType { JERK, SNAP };

/**
 * Generates time stamps for given waypoints given velocity and acceleration.
 **/
VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc);

/**
 * Ros2 node for trajectory generation.
 **/
class TrajGen : public rclcpp::Node {
    
public:
    /**
     * Initialises the Ros2 node.
     **/
    TrajGen();

private:

    /**
     * Generates a trajectory.
     **/
    void genTrajectory();

    
};