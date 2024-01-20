#include "lib/traj_min_jerk.hpp"
#include "lib/traj_min_snap.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>


using namespace std;
using namespace Eigen;

class RandomRouteGenerator
{
public:
    RandomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline MatrixXd generate(int N)
    {
        MatrixXd route(3, N + 1);
        Array3d temp;
        route.col(0).setZero();
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.col(i + 1) << temp;
        }
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};

VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

enum OptType { JERK, SNAP };

/**
 * Needs to produce a file which contains the target trajectory
 **/

int main(int argc, char **argv)
{
    // CONFIG ----
    RandomRouteGenerator routeGen(Array3d(-16, -16, -16), Array3d(16, 16, 16));
    int pieceNumber = 5;
    
    // Optimization type
    OptType optType = OptType::SNAP;

    // Target waypoints
    MatrixXd route;
    route = routeGen.generate(pieceNumber);

    // for jerk opt
    Matrix3d initialState, finalState;
    // Set initial position and final position, and set rest to zero
    initialState.setZero();
    finalState.setZero();
    initialState.col(0) << route.leftCols<1>();
    finalState.col(0) << route.rightCols<1>();

    // for snap opt
    Eigen::Matrix<double, 3, 4> initialStateSnap, finalStateSnap;
    // set initial and final pos and rest to zero
    initialStateSnap    << initialState, Eigen::MatrixXd::Zero(3, 1);
    finalStateSnap      << finalState, Eigen::MatrixXd::Zero(3, 1);
    
    VectorXd timestamps;
    timestamps = allocateTime(route, 3.0, 3.0);
    // -----------
    
    // SETUP -----

    // For aligning prints of Eigen vectors
    const IOFormat fmt(2, DontAlignCols, "\t", " ", "", "", "", "");

    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    // -----------

    // GEN ------- 
    
    // target waypoints in-between first and final position 
    auto pointsBetween = route.block(0, 1, 3, pieceNumber - 1);

    jerkOpt.reset(initialState, finalState, route.cols() - 1);
    jerkOpt.generate(pointsBetween, timestamps);
    jerkOpt.getTraj(minJerkTraj);

    snapOpt.reset(initialStateSnap, finalStateSnap, route.cols() - 1);
    snapOpt.generate(pointsBetween, timestamps);
    snapOpt.getTraj(minSnapTraj);
    // -----------
    
    // LOG -------
    // -----------
    return 0;
}