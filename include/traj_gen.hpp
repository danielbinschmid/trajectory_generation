#pragma once


#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>


using namespace Eigen;

enum OptType { JERK, SNAP };

VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc);

class TrajGen : public rclcpp::Node {
public:
    TrajGen();

private:

    void genTrajectory();

    
};