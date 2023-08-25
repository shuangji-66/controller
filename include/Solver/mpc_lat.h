
#pragma once
#include "Solver/solver_base.h"
#include <math.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "Eigen/LU"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Dense>
using CppAD::AD;
using Eigen::VectorXd;

using Matrix = Eigen::MatrixXd;

namespace ns_control
{
    struct xypoint
    {
        std::vector<double> x;
        std::vector<double> y;
    };
    class MPC_LAT_Solver : public Solver
    {
    public:
        void solve(ns_control::csvtraj trackingpath_);
        // 根据给定状态和轨迹的系数，返回控制变量(舵角度和速度)
        std::vector<double> Solve(Eigen::VectorXd state, const Eigen::VectorXd &coeffs, const xypoint &traj, const Weight &weight, std::vector<double> vel_flying);
        std::vector<double> x_pred_vals;
        std::vector<double> y_pred_vals;
        double curr_time = 0;
        double object_value_out = 0;
    };

}
