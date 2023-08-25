#include "Solver/mpc_lat.h"
#include "ros/ros.h"
#include <cmath>
#include "Solver/pid_controller.h"
#include<ctime>
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
    void MPC_LAT_Solver::solve(ns_control::csvtraj trackingpath_)
    {

        VectorXd coeffs;
        int index = findnearestindex(trackingpath_, state_);
  std::cout<<"index"<<index<<std::endl;
        std::vector<double> ref_vel;
        std::vector<double> xpoint;
        std::vector<double> ypoint;
        if (index < trackingpath_.size() - 40)
        {

            for (int i = 0; i < 40; ++i)
            {
                int new_index = i + index;
                ref_vel.push_back(trackingpath_[new_index].vel_flying + 2);
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
        xypoint ref_point;
        ref_point.x = xpoint;
        ref_point.y = ypoint;
        double px = state_.x;
        double py = state_.y;
        double psi = state_.yaw;
        double latency = 0.1; // 100 ms 计算MPC频率
        const double L = 1.58;
        double vx = state_.vx;
        px = px; // 世界坐标系下
        py = py;
        psi = psi;
        vx = vx + state_.D * latency;
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
        coeffs = polyfit(ptsxeig, ptsyeig, 5);

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
        Eigen::VectorXd state(8);
        state << (0+state_.vx*0.1), 0, 0, state_.vx, state_.vy, state_.r, 0, 0;
        // solve mpc for state and reference trajectory
        // returns [steering_angle, acceleration]
        Weight wei1;
        wei1.cte = control_param_.weight.cte;
        wei1.epsi = control_param_.weight.epsi;
        wei1.v = control_param_.weight.v;
        wei1.steer = control_param_.weight.cte;
        wei1.throttle = control_param_.weight.cte;
        wei1.steer_rate = control_param_.weight.cte;
        wei1.throttle_rate = control_param_.weight.cte;
        clock_t start_time=clock();
        auto actuations = Solve(state, coeffs, ref_point, wei1, ref_vel);
        clock_t endtime=clock();
        std::cout<<"mpc_time:  "<<(float)(endtime-start_time)/CLOCKS_PER_SEC<<std::endl;
        control_command_.steering_angle.data = actuations[0];
        double desire_vel = 4;
        auto vel = state_.v;
        PIDController speedpid(0.1, 0, 0);

        control_command_.throttle.data = speedpid.Control(5- vel, 0.1);

        // control_command_.throttle.data = actuations[1];
        std::cout << "******************youmen" << control_command_.throttle.data << std::endl;
        ROS_INFO_STREAM(control_command_.steering_angle.data);
        ROS_INFO_STREAM(control_command_.throttle.data);
    };
    using CppAD::AD;
    // 参考自:https://github.com/DhruvaKumar/model-predictive-control
    //  设置预测步长和控制周期，这里是10HZ一个MPC

    size_t N = 10;

    double dt = 0.1;
    const double L = 1.59;        // 这个是前轮轴心和后轮轴心的间距
    const double ref_v = 10;      // km/h 追踪轨迹
    const double ref_epsilon = 0; // 目标横向误差
    const double ref_sigma = 0;   // 目标偏航角误差
    // 由于非线性解释器把状态数据都存入一个向量中，以下定义会减轻后面代码量
    size_t x_start = 0; // x 坐标起始位置
    size_t y_start = x_start + N;
    size_t theta_start = y_start + N; // 偏航角
    size_t vx_start = theta_start + N;
    size_t vy_start = vx_start + N;
    size_t r_start = vy_start + N;
    size_t epsilon_start = r_start + N;     // 横向误差
    size_t sigma_start = epsilon_start + N; // 偏航误差
    size_t delta_start = sigma_start + N - 8;
    //  size_t D_start = delta_start + N - 1; // 加速度位置
    template <typename T>
    using Vec7 = Eigen::Matrix<T, 7, 1>;
    // 定义目标函数和约束函数

    class FG_eval
    {
    public:
        // 目标轨迹的系数
        Eigen::VectorXd coeffs;
        xypoint ref_point;
        VehicleState real_state;
        Weight wei_;
        std::vector<double> vel_flying;
        FG_eval(Eigen::VectorXd coeffs, Weight wei, std::vector<double> vel_flying, xypoint traj) : coeffs(coeffs), vel_flying(vel_flying)
        {
            this->wei_ = wei;
            this->ref_point = traj;
        };
        double Clf =88000;
        double Clr = 88000;
        double Sf = 0.2;
        double Sr = 0.2;
        // 前后轮侧偏刚度
        double Cf = 88000;
        double Cr = 88000;
        double Cm1 = 2145;
        double Cm2 = 175;
        double Cw = 0.5 * 1.206 * 0.5359 * 1.4;
        // 整车质量
        double m = 500;
        const double mass_fl = 233 / 4.0;            // 左前悬的质量
        const double mass_fr = 233 / 4.0;            // 右前悬的质量
        const double mass_rl = 233 / 4.0;            // 左后悬的质量
        const double mass_rr = 233 / 4.0;            // 右后悬的质量
        const double mass_front = mass_fl + mass_fr; // 前悬质量
        const double mass_rear = mass_rl + mass_rr;  // 后悬质量
        double mass_ = mass_front + mass_rear;

        double lf = L * (1.0 - mass_front / mass_); // 汽车前轮到中心点的距离
        double lr = L * (1.0 - mass_rear / mass_);  // 汽车后轮到中心点的距离

        // moment of inertia
        // double Iz = lf * lf * mass_front + lr * lr * mass_rear; // 汽车的转动惯量
        double Iz = 5000;
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

            // cost_weight << wei_.cte, wei_.epsi,
            //     wei_.v, wei_.steer, wei_.throttle,
            //     wei_.steer_rate, wei_.throttle_rate;
            cost_weight << 10, 40, 100, 1000, 100, 10000, 100;
            // 代价函数的目标，最小化横向误差，最小化偏航误差，最小化速度，加速度误差，同时要满足转向速度和加速度要相对平滑
            // 不能出现大的跳跃
            // fg[0]默认是优化目标函数
            // // 整个周期都需要的
            for (int t = 0; t < N; t++)
            {

                fg[0] += cost_weight[0] * CppAD::pow(x_state[epsilon_start + t] - ref_epsilon, 2);
                fg[0] += cost_weight[1] * CppAD::pow(x_state[sigma_start + t] - ref_sigma, 2);
                //    fg[0] += cost_weight[0] * CppAD::pow(x_state[x_start + t] - ref_point.x[t], 2);
                //   fg[0] += cost_weight[1] * CppAD::pow(x_state[y_start + t] - ref_point.y[t], 2);
                // fg[0] += cost_weight[2] * CppAD::pow(x_state[vx_start + t] - ref_v, 2);
            }
            // 计算最小系统输入
            for (int t = 0; t < N - 8; t++)
            {
                fg[0] += cost_weight[3] * CppAD::pow(x_state[delta_start + t], 2);
                // fg[0] += cost_weight[4] * CppAD::pow(x_state[D_start + t], 2);
            }
            // 计算系统输入之间的变量误差不能太大，相当于方差不能太大，可以这么理解。
            for (int t = 0; t < N - 9; t++)
            {
                fg[0] += cost_weight[5] * CppAD::pow(x_state[delta_start + t + 1] - x_state[delta_start + t], 2);
                //    fg[0] += cost_weight[6] * CppAD::pow(x_state[D_start + t + 1] - x_state[D_start + t], 2);
            }
            //------------------------ model constraints
            // subject to
            // 初始化 参数，正常来说应该是0.注意这里和公式里推的做了一个track全部转成等式，目标值为0
            fg[1 + x_start] = x_state[x_start];
            fg[1 + y_start] = x_state[y_start];
            fg[1 + theta_start] = x_state[theta_start];
            fg[1 + vx_start] = x_state[vx_start];
            fg[1 + vy_start] = x_state[vy_start];
            fg[1 + r_start] = x_state[r_start];
            fg[1 + epsilon_start] = x_state[epsilon_start];
            fg[1 + sigma_start] = x_state[sigma_start];

            for (int t = 1; t < N; t++)
            {
                // The state at time t+1 .下一个状态
                AD<double> x1 = x_state[x_start + t];
                AD<double> y1 = x_state[y_start + t];
                AD<double> theta1 = x_state[theta_start + t];
                AD<double> vx1 = x_state[vx_start + t] + 0.0001;
                AD<double> vy1 = x_state[vy_start + t];
                AD<double> r1 = x_state[r_start + t];
                AD<double> epsilon1 = x_state[epsilon_start + t];
                AD<double> sigma1 = x_state[sigma_start + t];
                // 当前时间的状态
                AD<double> x0 = x_state[x_start + t - 1];
                AD<double> y0 = x_state[y_start + t - 1];
                AD<double> theta0 = x_state[theta_start + t - 1];
                AD<double> vx0 = x_state[vx_start + t - 1] + 0.0001;
                AD<double> vy0 = x_state[vy_start + t - 1];
                AD<double> r0 = x_state[r_start + t - 1];
                AD<double> epsilon0 = x_state[epsilon_start + t - 1];
                AD<double> sigma0 = x_state[sigma_start + t - 1];
                // 只考虑当前的控制变量的状态
                AD<double> delta0 = x_state[delta_start + t - 1];
                //    AD<double> D0 = x_state[D_start + t - 1];
                //  拟合的目标轨迹曲线
                AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0+coeffs[4] *x0* x0 * x0 * x0+coeffs[5] *x0*x0* x0 * x0 * x0;
                // 一阶导，用来计算偏航误差
                AD<double> f0prime = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0+ 4 * coeffs[4] *x0* x0 * x0+ 5 * coeffs[5]*x0 * x0 * x0;
                // 默认一开始是没有偏航误差，所以这里可以认为直接等于一阶导在初始值的值
                AD<double> delta_e0 = CppAD::atan(f0prime);
                // equations for the model:

                fg[1 + x_start + t] =
                    x1 - (x0 + vx0 * CppAD::cos(theta0) * dt - vy0 * CppAD::sin(theta0) * dt);

                fg[1 + y_start + t] =
                    y1 - (y0 + vx0 * CppAD::sin(theta0) * dt + vy0 * CppAD::cos(theta0) * dt);

                fg[1 + theta_start + t] =
                    theta1 - (theta0 + r0 * dt); // 这里主要是为了和仿真平台相对应

                //AD<double> acc = vy0 * r0 + 1 / m * (Clf * Sf + Cf * (-delta0 - ((vy0 + lf * r0) / vx0)) * -delta0 + Clr * Sr);
                // AD<double> Ffy = Cf * ((vy0 + lf * -r0) / vx0 - delta0);

                // AD<double> acc = ((Cm1 - Cm2 * vx0) * D0 - Cw * vx0 * vx0 - 286.65 - Ffy * sin(delta0) + m * vy0 * -theta0) / m;
                fg[1 + vx_start + t] = vx1 - (vx0 + (vy0 * r0 + 1 / m * (Clf * Sf + Cf * (-delta0 - ((vy0 + lf * r0) / vx0)) * -delta0 + Clr * Sr)) * dt);


                fg[1 + vy_start + t] =  vy1 - (vy0 + (  -(vx0 * r0) + (1 / m) * (Cf * (-delta0 - (vy0 + lf * r0) / vx0) + Cr * ((lr * r0 - vy0) / vx0)))* dt);

                AD<double> yaw_acc =
                    (1 / Iz) * ((lf * Cf * (-delta0 - ((vy0 + lf * r0) / vx0))) - (lr * Cr * (lr * r0 - vy0) / vx0));

                fg[1 + r_start + t] =
                    r1 - (r0 + yaw_acc * dt);

                fg[1 + epsilon_start + t] =
                    epsilon1 - ((f0 - y0) + (vx0 * CppAD::sin(sigma0) * dt));

                fg[1 + sigma_start + t] =
                    sigma1 - ((theta0 - delta_e0) - vx0 * delta0 / L* dt); // 这里也是一样的右边乘了一个负号
            }
        }
    };

    std::vector<double> MPC_LAT_Solver::Solve(Eigen::VectorXd state, const Eigen::VectorXd &coeffs, const xypoint &traj, const Weight &weight, std::vector<double> vel_flying)
    {
        // 使用ipopt求解
        bool ok = true;
        typedef CPPAD_TESTVECTOR(double) Dvector;
        // 运行时间，用于画图
        curr_time += dt;
        // 存储状态变量的个数，主要是8个状态变量和两个输出变量
        size_t n_state = 8 * N + 1 * (N - 8);
        size_t n_constraints = 8 * N; // constriants
        Dvector vars(n_state);
        for (int i = 0; i < n_state; i++)
            vars[i] = 0;
        // set initial state
        vars[x_start] = state[0];
        vars[y_start] = state[1];
        vars[theta_start] = state[2];
        vars[vx_start] = state[3];
        vars[vy_start] = state[4];
        vars[r_start] = state[5];
        vars[epsilon_start] = state[6];
        vars[sigma_start] = state[7];
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
        // for (int i = delta_start; i < D_start; i++)
        // {
        //     vars_lowerbound[i] = -0.436;
        //     vars_upperbound[i] = 0.436;
        // }
        for (int i = delta_start; i < n_state; i++)
        {
            vars_lowerbound[i] = -0.5;
            vars_upperbound[i] = 0.5;
        }
        // 车油门的范围，也就是车加速度
        // for (int i = D_start; i < n_state; i++)
        // {
        //     vars_lowerbound[i] = -1.0;
        //     vars_upperbound[i] = 1.0;
        // }
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
        constraints_lowerbound[vx_start] = state[3];
        constraints_lowerbound[vy_start] = state[4];
        constraints_lowerbound[r_start] = state[5];
        constraints_lowerbound[epsilon_start] = state[6];
        constraints_lowerbound[sigma_start] = state[7];

        constraints_upperbound[x_start] = state[0];
        constraints_upperbound[y_start] = state[1];
        constraints_upperbound[theta_start] = state[2];
        constraints_upperbound[vx_start] = state[3];
        constraints_upperbound[vy_start] = state[4];
        constraints_upperbound[r_start] = state[5];
        constraints_upperbound[epsilon_start] = state[6];
        constraints_upperbound[sigma_start] = state[7];
        // object that computes objective and constraints
        // 建立一个f和g相关的对象，把拟合的
        FG_eval fg_eval(coeffs, weight, vel_flying, traj);
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
        // std::cout << "obj_value " << cost << std::endl;
        object_value_out = cost;
        //    std::cout<<"Res"<<solution.x<<"object value"<<solution.obj_value<<"zl"<<solution.zl<<"zu"<<solution.zu<<"g"<<solution.g<<"lambda"<<solution.lambda
        //             <<std::endl;
        // 下面主要是存储每次计算出来的轨迹，用于画图
        for (int i = 0; i < N; i++)
        {
            geometry_msgs::Point32 p;
            p.x = solution.x[x_start + i];
            p.y = solution.x[y_start + i];
        }

        predictive_path.clear();
        TrajectoryPoint p_tmp;
        for (int i = 0; i < N; i++)
        {
            geometry_msgs::Point32 p;
            p_tmp.pts.x = solution.x[x_start + i];
            p_tmp.pts.y = solution.x[y_start + i];
            p_tmp.velocity = solution.x[vx_start + i];
            predictive_path.push_back(p_tmp);
        }
        // 返回 输出给车的变量
        // return {solution.x[delta_start], solution.x[D_start]};
        return {solution.x[delta_start]};
    }

}

// std::vector<double> x_delta;
// std::vector<double> y_delta;
// std::vector<double> yaw_delta;
// for (int t = 0; t < N; t++)
// {
//     double xx = traj[t].pts.x - real_state.x;
//     double yy = traj[t].pts.y - real_state.y;
//     double temp_x = xx * cos(real_state.yaw) + yy * sin(real_state.yaw);
//     double temp_y = yy * cos(real_state.yaw) - xx * sin(real_state.yaw);
//     x_delta.push_back(temp_x);
//     y_delta.push_back(temp_y);
//     double dyaw = traj[t].yaw - real_state.yaw;
//     while ((dyaw) >= M_PI)
//         dyaw -= M_PI * 2.0;
//     while ((dyaw) <= -1.0 * M_PI)
//         dyaw += M_PI * 2.0;
//     yaw_delta.push_back(dyaw);
// }
// std::cout << "**************x*********************" << x_delta[0] << std::endl;
// std::cout << "**************y*********************" << y_delta[0] << std::endl;
// fg[0] += cost_weight[0] * CppAD::pow(x_state[x_start + t] - x_delta[t], 2);
// fg[0] += cost_weight[1] * CppAD::pow(x_state[y_start + t] - y_delta[t], 2);
// fg[0] += 1000* CppAD::pow(x_state[theta_start + t] - yaw_delta[t], 2);

/*两个版本*/
// AD<double> thetavf = CppAD::atan((vy0 + lf * r0) / vx0);
// AD<double> thetavr = CppAD::atan((vy0 - lr * r0) / vx0);
// AD<double> Fyf = 2 * Cf * (delta0 - thetavf);
// AD<double> Fyr = 2 * Cr * (-thetavr);
// fg[1 + vy_start + t] = vy1 - (vy0 + (1 / m * (Fyr - Fyf * CppAD::cos(delta0) - m * vx0 * r0)) * dt);
// fg[1 + r_start + t] = r1 - (r0 + (1 / (Iz/10) * (Fyf * lf * CppAD::cos(delta0) - Fyr * lr)) * dt);
/*两个版本*/