/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2021:
     - chentairan <tairanchen@bitfsd.cn>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Solver/mpc_kin_solver.h"
#include "ros/ros.h"
#include <cmath>

namespace ns_control
{

    // 计算多项式的值
    double polyeval(Eigen::VectorXd coeffs, double x)
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++)
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    // 拟合一条曲线
    //  Fit a polynomial.
    //  Adapted from
    //  https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                            int order)
    {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
        {
            A(i, 0) = 1.0;
        }

        for (int j = 0; j < xvals.size(); j++)
        {
            for (int i = 0; i < order; i++)
            {
                A(j, i + 1) = A(j, i) * xvals(j);
            }
        }

        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    // 两点之间的距离
    double PointDistanceSquare(const ns_control::CsvPoint &point, const double x,
                               const double y)
    {
        double dx = point.pointx - x;
        double dy = point.pointy - y;
        return dx * dx + dy * dy;
    }
    
    int findnearestindex(const ns_control::csvtraj &path, VehicleState state_)
    {
        int index = 0;
        double d_min = PointDistanceSquare(path.front(), state_.x, state_.y);
        for (int i = 1; i < path.size(); ++i)
        {

            double d_temp = PointDistanceSquare(path[i], state_.x, state_.y);
            if (d_temp < d_min)
            {
                d_min = d_temp;
                index = i;
            }
        }

        return index;
    }
    void MPC_KIN_Solver::solve(ns_control::csvtraj trackingpath_)
    {
        ROS_INFO_STREAM("begin solve");
        ROS_INFO_STREAM("MPC_KIN_SOLVER");
        // if (trajectory_.empty())
        // {
        //     control_command_.throttle.data = static_cast<float>(-1.0); // 类型转换
        //     control_command_.steering_angle.data = 0.0;
        //     ROS_INFO_STREAM("trajectory empty");
        //     return;
        // }

        std::cout << "trackingpath.size" << trackingpath_.size() << std::endl;
        VectorXd coeffs;
        int index = findnearestindex(trackingpath_, state_);

        std::cout << "find!!!!!!!!!" << index << std::endl;
        std::vector<double> ref_vel;
        std::vector<double> xpoint;
        std::vector<double> ypoint;
        if (index < trackingpath_.size() - 40)
        {

            for (int i = 0; i < 40; ++i)
            {
                int new_index = i + index;
                ref_vel.push_back(trackingpath_[new_index].vel_flying+2);
                xpoint.push_back(trackingpath_[new_index].pointx);
                ypoint.push_back(trackingpath_[new_index].pointy);
            }
        }

        else
        {
            for (int i = 0; i < trackingpath_.size() - index; ++i)
            {
                ref_vel.push_back(trackingpath_[i + index].vel_flying);
                xpoint.push_back(trackingpath_[i + index].pointx);
                ypoint.push_back(trackingpath_[i + index].pointy);
            }
            for (int i = 0; i < 40 - (trackingpath_.size() - index); ++i)
            {
                ref_vel.push_back(trackingpath_[i].vel_flying);
                xpoint.push_back(trackingpath_[i].pointx);
                ypoint.push_back(trackingpath_[i].pointy);
            }
        }
        std::cout << "ref_vel" << ref_vel[1] << std::endl;

        double px = state_.x;
        double py = state_.y;
        double psi = state_.yaw;
        double latency = 0.1; // 100 ms 计算MPC频率
        const double L = 1.58;
        double v = state_.v;
        px = px; // 世界坐标系下
        py = py;
        psi = psi;
        v = v + state_.D * latency;
        // unsigned int reference_path_length = 12;
        std::vector<double> ptsx;
        std::vector<double> ptsy;
        for (int i = 0; i < xpoint.size(); ++i)
        {
            auto xdiff = (xpoint[i] - px);
            auto ydiff = (ypoint[i] - py);

            ptsx.push_back(xdiff * cos(-psi) + ydiff * sin(psi));
            ptsy.push_back(ydiff * cos(-psi) - xdiff * sin(psi));
        }
        // 从全局路径中，找到距离当前位置最近的前方的点。
        double former_point_of_current_position = 0;
        for (size_t i = former_point_of_current_position; i < ptsx.size(); i++)
        {
            if (ptsx[i] > 0.0)
            {
                former_point_of_current_position = i;
                break;
            }
        }
        // calculate coeffs of reference trajectory polynomial
        // 计算系数矩阵
        Eigen::Map<Eigen::VectorXd> ptsxeig(&ptsx[former_point_of_current_position], 20);
        Eigen::Map<Eigen::VectorXd> ptsyeig(&ptsy[former_point_of_current_position], 20);
        coeffs = polyfit(ptsxeig, ptsyeig, 3);

        // calculate the cross track error
        double cte = polyeval(coeffs, 0);
        // calculate the orientation error
        // f(x) is the polynomial defining the reference trajectory
        // f'(x) = 3Ax^2 + 2Bx + C
        // double f_prime_x = 3*coeffs[3]*pow(px,2) + 2*coeffs[2]*px + coeffs[1];
        double f_prime_x = coeffs[1];
        double epsi = -atan(f_prime_x);

        // 在车辆坐标系下，当前时刻的位置偏差就是期望轨迹在车辆坐标系中的截距

        // state
        Eigen::VectorXd state(6);
        state << 0, 0, 0, v, 0, 0;
        // solve mpc for state and reference trajectory
        // returns [steering_angle, acceleration]
        auto actuations = Solve(state, coeffs, trajectory_, ref_vel, xpoint,
                                ypoint);

        control_command_.steering_angle.data = actuations[0];

        control_command_.throttle.data = actuations[1];
    };
    using CppAD::AD;
    // 参考自:https://github.com/DhruvaKumar/model-predictive-control
    //  设置预测步长和控制周期，这里是10HZ一个MPC
    size_t N = 10;
    double dt = 0.1;
    const double L = 1.88;        // 这个是前轮轴心和后轮轴心的间距
    const double ref_v = 9;       // km/h 追踪轨迹
    const double ref_epsilon = 0; // 目标横向误差
    const double ref_sigma = 0;   // 目标偏航角误差
    // 由于非线性解释器把状态数据都存入一个向量中，以下定义会减轻后面代码量
    size_t x_start = 0; // x 坐标起始位置
    size_t y_start = x_start + N;
    size_t theta_start = y_start + N; // 偏航角
    size_t v_start = theta_start + N;
    size_t epsilon_start = v_start + N;     // 横向误差
    size_t sigma_start = epsilon_start + N; // 偏航误差
    size_t delta_start = sigma_start + N;
    // size_t delta_start = v_start + N;
    size_t a_start = delta_start + N - 1; // 加速度位置
    template <typename T>
    using Vec7 = Eigen::Matrix<T, 7, 1>;
    // 定义目标函数和约束函数

    class FG_eval
    {
    public:
        // 目标轨迹的系数
        Eigen::VectorXd coeffs;
        Trajectory traj;
        std::vector<double> vel_flying;
        std::vector<double> xpoint;
        std::vector<double> ypoint;
        FG_eval(Eigen::VectorXd coeffs, Trajectory traj, std::vector<double> vel_flying, std::vector<double> xpoint,
                std::vector<double> ypoint) : coeffs(coeffs), traj(traj), vel_flying(vel_flying), xpoint(xpoint), ypoint(ypoint){};

        // 7x1 Vector
        Vec7<int> cost_weight;
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        // 函数重载，ipopt要求
        void operator()(ADvector &fg, const ADvector &x_state)
        {
            // fg 是所有等式或者不等式，包括目标优化函数的式子，每一个index存储一个式子，这里不包含
            // 状态的上下界，这个单独算到constraints
            fg[0] = 0;
            // cost function的系数，这个主要是手动调整
            //         weight:
            //   px: 1.5
            //   py: 1.5
            //   pyaw: 8
            //   cte: 1
            //   epsi: 4
            //   v: 0.4
            //   steer: 4
            //   throttle: 10
            //   steer_rate: 2000
            //   throttle_rate: 10
            //   desire_vel: 15

            cost_weight << 100, 100, 10000, 50, 1000, 1000, 10;
            // 代价函数的目标，最小化横向误差，最小化偏航误差，最小化速度，加速度误差，同时要满足转向速度和加速度要相对平滑
            // 不能出现大的跳跃
            // fg[0]默认是优化目标函数
            // 整个周期都需要的
            for (int t = 0; t < N; t++)
            {
                fg[0] += cost_weight[0] * CppAD::pow(x_state[epsilon_start + t] - ref_epsilon, 2);
                fg[0] += cost_weight[1] * CppAD::pow(x_state[sigma_start + t] - ref_sigma, 2);
                // fg[0]+=cost_weight[2]*CppAD::pow(x_state[v_start+t]-ref_v-1,2);
                // fg[0] += cost_weight[0] * CppAD::pow(x_state[x_start + t] -xpoint[t] , 2);
                // fg[0] += cost_weight[1] * CppAD::pow(x_state[y_start + t] -ypoint[t], 2);
                // AD<double>yaw_error=x_state[theta_start+t]-traj[t].yaw;
                //             if (x_state[theta_start+t]-traj[t].yaw> M_PI) x_state[theta_start+t]-traj[t].yaw -= 2 * M_PI;
                // if (x_state[theta_start+t]-traj[t].yaw < -M_PI) x_state[theta_start+t]-traj[t].yaw += 2 * M_PI;
                // fg[0]+=0*CppAD::pow(x_state[theta_start+t]-traj[t].yaw,2);
                fg[0] += cost_weight[2] * CppAD::pow(x_state[v_start + t] - vel_flying[t], 2);
            }
            // 计算最小系统输入
            for (int t = 0; t < N - 1; t++)
            {
                fg[0] += cost_weight[3] * CppAD::pow(x_state[delta_start + t], 2);
                fg[0] += cost_weight[4] * CppAD::pow(x_state[a_start + t], 2);
            }
            // 计算系统输入之间的变量误差不能太大，相当于方差不能太大，可以这么理解。
            for (int t = 0; t < N - 2; t++)
            {
                fg[0] += cost_weight[5] * CppAD::pow(x_state[delta_start + t + 1] - x_state[delta_start + t], 2);
                fg[0] += cost_weight[6] * CppAD::pow(x_state[a_start + t + 1] - x_state[a_start + t], 2);
            }
            //------------------------ model constraints
            // subject to
            // 初始化 参数，正常来说应该是0.注意这里和公式里推的做了一个track全部转成等式，目标值为0
            fg[1 + x_start] = x_state[x_start];
            fg[1 + y_start] = x_state[y_start];
            fg[1 + theta_start] = x_state[theta_start];
            fg[1 + v_start] = x_state[v_start];
            fg[1 + epsilon_start] = x_state[epsilon_start];
            fg[1 + sigma_start] = x_state[sigma_start];

            for (int t = 1; t < N; t++)
            {
                // The state at time t+1 .下一个状态
                AD<double> x1 = x_state[x_start + t];
                AD<double> y1 = x_state[y_start + t];
                AD<double> theta1 = x_state[theta_start + t];
                AD<double> v1 = x_state[v_start + t];
                AD<double> epsilon1 = x_state[epsilon_start + t];
                AD<double> sigma1 = x_state[sigma_start + t];
                // 当前时间的状态
                AD<double> x0 = x_state[x_start + t - 1];
                AD<double> y0 = x_state[y_start + t - 1];
                AD<double> theta0 = x_state[theta_start + t - 1];
                AD<double> v0 = x_state[v_start + t - 1];
                AD<double> epsilon0 = x_state[epsilon_start + t - 1];
                AD<double> sigma0 = x_state[sigma_start + t - 1];
                // 只考虑当前的控制变量的状态
                AD<double> delta0 = x_state[delta_start + t - 1];
                AD<double> a0 = x_state[a_start + t - 1];
                // 拟合的目标轨迹曲线
                AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
                // 一阶导，用来计算偏航误差
                AD<double> f0prime = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0;
                // 默认一开始是没有偏航误差，所以这里可以认为直接等于一阶导在初始值的值
                AD<double> delta_e0 = CppAD::atan(f0prime);
                //  equations for the model:
                //  0=x_[t] -x[t-1] - v[t-1] * cos(psi[t-1]) * dt
                //  0=y_[t] - y[t-1] - v[t-1] * sin(psi[t-1]) * dt
                //  0=theta_[t] -theta[t-1] - v[t-1] / Lf * delta[t-1] * dt
                //  0=v_[t] - v[t-1] - a[t-1] * dt
                //  0=epsilon[t] - f(x[t-1]) + y[t-1] -1 v[t-1] * sin(epsi[t-1]) * dt
                //  0=sigma[t] - theta[t] - delta_e[t-1] - v[t-1] * delta[t-1] / Lf * dt
                fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(theta0) * dt);
                fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(theta0) * dt);
                fg[1 + theta_start + t] = theta1 - (theta0 - v0 * -delta0 / L * dt); // 这里主要是为了和仿真平台相对应
                fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
                fg[1 + epsilon_start + t] =
                    epsilon1 - ((f0 - y0) + (v0 * CppAD::sin(sigma0) * dt));
                fg[1 + sigma_start + t] =
                    sigma1 - ((theta0 - delta_e0) - v0 *- delta0 / L * dt); // 这里也是一样的右边乘了一个负号
            }
        }
    };

    std::vector<double> MPC_KIN_Solver::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, Trajectory traj, std::vector<double> vel_flying, std::vector<double> xpoint,
                                              std::vector<double> ypoint)
    {
        // 使用ipopt求解
        bool ok = true;
        typedef CPPAD_TESTVECTOR(double) Dvector;
        // 运行时间，用于画图
        curr_time += dt;
        // 存储状态变量的个数，主要是6个状态变量和两个输出变量
        size_t n_state = 6 * N + 2 * (N - 1);
        size_t n_constraints = 6 * N; // constriants
        Dvector vars(n_state);
        for (int i = 0; i < n_state; i++)
            vars[i] = 0;
        // set initial state
        vars[x_start] = state[0];
        vars[y_start] = state[1];
        vars[theta_start] = state[2];
        vars[v_start] = state[3];
        vars[epsilon_start] = state[4];
        vars[sigma_start] = state[5];
        // Set lower and upper limits for variables. 设置 变量的值范围
        Dvector vars_lowerbound(n_state);
        Dvector vars_upperbound(n_state);
        // Set all non-actuators upper and lowerlimits
        // to the max negative and positive values.
        // 设置 除了车的控制输入以外的变量为最大值
        for (int i = 0; i < delta_start; i++)
        {
            vars_lowerbound[i] = -1.0e19;
            vars_upperbound[i] = 1.0e19;
        }
        // 这里设置方向盘最大最小转角为正负25度
        for (int i = delta_start; i < a_start; i++)
        {
            vars_lowerbound[i] = -0.8;
            vars_upperbound[i] = 0.8;
        }
        // 车油门的范围，也就是车加速度
        for (int i = a_start; i < n_state; i++)
        {
            vars_lowerbound[i] = -1.0;
            vars_upperbound[i] = 1.0;
        }
        //
        // Lower and upper limits for the constraints
        // Should be 0 besides initial state.
        // 也就是g(x)这些的范围，要都为0，因为我们已经转化为减法了
        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);
        for (int i = 0; i < n_constraints; i++)
        {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
        }
        // 这里一会来看看？
        constraints_lowerbound[x_start] = state[0];
        constraints_lowerbound[y_start] = state[1];
        constraints_lowerbound[theta_start] = state[2];
        constraints_lowerbound[v_start] = state[3];
        constraints_lowerbound[epsilon_start] = state[4];
        constraints_lowerbound[sigma_start] = state[5];

        constraints_upperbound[x_start] = state[0];
        constraints_upperbound[y_start] = state[1];
        constraints_upperbound[theta_start] = state[2];
        constraints_upperbound[v_start] = state[3];
        constraints_upperbound[epsilon_start] = state[4];
        constraints_upperbound[sigma_start] = state[5];
        // object that computes objective and constraints
        // 建立一个f和g相关的对象，把拟合的
        FG_eval fg_eval(coeffs, trajectory_, vel_flying, xpoint, ypoint);
        // 下面是配置 iopt求解器的配置
        std::string options;
        //    std::cout<<vars.size()<<vars_lowerbound.size()<<std::endl;
        // Uncomment this if you'd like more print information
        options += "Integer print_level  0\n";
        // NOTE: Setting sparse to true allows the solver to take advantage
        // of sparse routines, this makes the computation MUCH FASTER. If you
        // can uncomment 1 of these and see if it makes a difference or not but
        // if you uncomment both the computation time should go up in orders of
        // magnitude.
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.
        options += "Numeric max_cpu_time          0.5\n";
        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;
        // 调用求解器求解
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);
        // 看看有木有求解成功
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
        // Cost
        auto cost = solution.obj_value;
        std::cout << "obj_value " << cost << std::endl;
        object_value_out = cost;
        //    std::cout<<"Res"<<solution.x<<"object value"<<solution.obj_value<<"zl"<<solution.zl<<"zu"<<solution.zu<<"g"<<solution.g<<"lambda"<<solution.lambda
        //             <<std::endl;
        // 下面主要是存储每次计算出来的轨迹，用于画图
        x_pred_vals.clear();
        y_pred_vals.clear();
        for (int i = 1; i < N; ++i)
        {
            x_pred_vals.push_back(solution.x[x_start + i]);
            y_pred_vals.push_back(solution.x[y_start + i]);
        }
        // 返回 输出给车的变量
        return {solution.x[delta_start], solution.x[a_start]};
    }

}
