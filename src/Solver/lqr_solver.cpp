#include "Solver/lqr_solver.h"
#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>
#include "Solver/pid_controller.h"
#include "Eigen/LU"
#include "math.h"

namespace ns_control
{
    // 两点之间的距离
    double LQR_SOLVER::PointDistanceSquare(const ns_control::CsvPoint &point, const double x,
                                           const double y)
    {
        double dx = point.pointx - x;
        double dy = point.pointy - y;
        return dx * dx + dy * dy;
    }
    int LQR_SOLVER::findnearestindex(const ns_control::csvtraj &path, VehicleState state_)
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
    void LQR_SOLVER::solve(ns_control::csvtraj trackingpath_)
    {
        ROS_INFO_STREAM("begin solve");
        // if (this->trajectory_.empty())
        // {
        //     control_command_.throttle.data = static_cast<float>(-1.0);
        //     control_command_.steering_angle.data = 0.0;
        //     ROS_INFO_STREAM("trajectory empty");
        //     return;
        // }
        // std::cout<<"planning speed"<<trajectory_[1].velocity<<std::endl;
        // std::cout<<"real speed"<<state_.v<<std::endl;
        double desire_vel = control_param_.desire_vel;
        LoadContolConf();
        init();
        int index = findnearestindex(trackingpath_, state_);

        std::cout << "find!!!!!!!!!" << index << std::endl;
        std::vector<double> ref_vel;
        std::vector<double> xpoint;
        std::vector<double> ypoint;
        std::vector<double> yaw;
        if (index < trackingpath_.size() - 40)
        {

            for (int i = 0; i < 40; ++i)
            {
                int new_index = i + index;
                ref_vel.push_back(trackingpath_[new_index].vel_flying + 2);
                xpoint.push_back(trackingpath_[new_index].pointx);
                ypoint.push_back(trackingpath_[new_index].pointy);
                yaw.push_back(trackingpath_[new_index].pointyaw);
            }
        }

        else
        {
            for (int i = 0; i < trackingpath_.size() - index; ++i)
            {
                ref_vel.push_back(trackingpath_[i + index].vel_flying);
                xpoint.push_back(trackingpath_[i + index].pointx);
                ypoint.push_back(trackingpath_[i + index].pointy);
                yaw.push_back(trackingpath_[i + index].pointyaw);
            }
            for (int i = 0; i < 40 - (trackingpath_.size() - index); ++i)
            {
                ref_vel.push_back(trackingpath_[i].vel_flying);
                xpoint.push_back(trackingpath_[i].pointx);
                ypoint.push_back(trackingpath_[i].pointy);
                yaw.push_back(trackingpath_[i].pointyaw);
            }
        }
        Trajectory traj_new;
        TrajectoryPoint traj_point;
        for (int i = 0; i < ref_vel.size(); ++i)
        {
            traj_point.pts.x = xpoint[i];
            traj_point.pts.y = ypoint[i];
            traj_point.yaw = yaw[i];
            traj_point.velocity = ref_vel[i];
            traj_point.acc = 0;
            traj_point.curvature = 0;
            traj_point.r = 0;
            traj_new.push_back(traj_point);
        }
         std::cout << "LQR failed" << std::endl;
        if (!ComputeControlCommand(state_, traj_new, control_command_))
            std::cout << "LQR failed" << std::endl;
        // std::cout << "steer data" << control_command_.steering_angle.data << std::endl;
        // Speed Controller
        PIDController speedpid(0.05, 0.1, 0);
        const double vel = state_.v;
        //  control_command_.throttle.data = static_cast<float>(7 - vel);

        control_command_.throttle.data = speedpid.Control(ref_vel[2] - vel, 0.1);
        // std::cout << control_command_.throttle.data << "********" << std::endl;
    }

    void LQR_SOLVER::LoadContolConf()
    {

        ts_ = 0.01; // 每隔0.01s进行一次控制

        cf_ = 88539.01;                           // 前轮侧偏刚度,左右轮之和
        cr_ = 88539.01;                           // 后轮侧偏刚度, 左右轮之和
        wheelbase_ = 1.88;                        // 左右轮的距离
        steer_ratio_ = 10;                        // 方向盘的转角到轮胎转动角度之间的比值系数
        steer_single_direction_max_degree_ = 180; // 最大方向转角

        const double mass_fl = 75;                   // 左前悬的质量
        const double mass_fr = 75;                   // 右前悬的质量
        const double mass_rl = 75;                   // 左后悬的质量
        const double mass_rr = 75;                   // 右后悬的质量
        const double mass_front = mass_fl + mass_fr; // 前悬质量
        const double mass_rear = mass_rl + mass_rr;  // 后悬质量
        mass_ = mass_front + mass_rear;

        lf_ = wheelbase_ * (1.0 - mass_front / mass_); // 汽车前轮到中心点的距离
        lr_ = wheelbase_ * (1.0 - mass_rear / mass_);  // 汽车后轮到中心点的距离

        // moment of inertia
        iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear; // 汽车的转动惯量

        lqr_eps_ = 0.01;           // LQR 迭代求解精度
        lqr_max_iteration_ = 1500; // LQR的迭代次数

        return;
    }
    void LQR_SOLVER::init()
    {

        // Matrix init operations.
        const int matrix_size = basic_state_size_;
        matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
        matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
        /*
      A matrix (Gear Drive)
      [0.0, 1.0, 0.0, 0.0;
      0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
      (l_r * c_r - l_f * c_f) / m / v;
      0.0, 0.0, 0.0, 1.0;
      0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
      (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
      */
        // 初始化A矩阵的常数项
        matrix_a_(0, 1) = 1.0;
        matrix_a_(1, 2) = (cf_ + cr_) / mass_;
        matrix_a_(2, 3) = 1.0;
        matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

        // 初始化A矩阵的非常数项
        matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
        matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
        matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
        matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
        matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

        /*
      b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
      */
        // 初始化B矩阵
        matrix_b_ = Matrix::Zero(basic_state_size_, 1);
        matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        matrix_bd_ = matrix_b_ * ts_;

        // 状态向量
        matrix_state_ = Matrix::Zero(matrix_size, 1);
        // 反馈矩阵
        matrix_k_ = Matrix::Zero(1, matrix_size);
        // lqr cost function中 输入值u的权重
        matrix_r_ = Matrix::Identity(1, 1);
        matrix_r_(0, 0) = 10;
        // lqr cost function中 状态向量x的权重
        matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

        // int q_param_size = 4;
        matrix_q_(0, 0) = 10; // lateral_error
        matrix_q_(1, 1) = 10; // lateral_error_rate
        matrix_q_(2, 2) = 1;  // heading_error
        matrix_q_(3, 3) = 1;  // heading__error_rate

        matrix_q_updated_ = matrix_q_;

        return;
    }

    // 将角度(弧度制)归化到[-M_PI, M_PI]之间
    double NormalizeAngle(const double angle)
    {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    bool LQR_SOLVER::ComputeControlCommand(const VehicleState &localization,
                                           const Trajectory &planning_published_trajectory,
                                           fsd_common_msgs::ControlCommand &cmd)
    {

        double v_ = localization.v > this->minimum_speed_protection_ ? localization.v : this->minimum_speed_protection_;
        // double v_=0.1;
        this->matrix_a_(1, 1) = this->matrix_a_coeff_(1, 1) / v_;
        this->matrix_a_(1, 3) = this->matrix_a_coeff_(1, 3) / v_;
        this->matrix_a_(3, 1) = this->matrix_a_coeff_(3, 1) / v_;
        this->matrix_a_(3, 3) = this->matrix_a_coeff_(3, 3) / v_;

        /*
        b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
        */
        // to-do 02 动力矩阵B

        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        this->matrix_bd_ = this->matrix_b_ * this->ts_;
        // std::cout
        //  << "matrix_bd_.row(): " << matrix_bd_.rows() << std::endl;
        // std::cout << "matrix_bd_.col(): " << matrix_bd_.cols() << std::endl;
        //   Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
        //   Error Rate]
        std::cout<<"aaaaaaaaaaaaaaaaaaa"<<std::endl;
        // to-do 03 计算横向误差并且更新状态向量x
        UpdateState(localization,planning_published_trajectory);
 std::cout<<"bbbbbbbbbbbbbbbbbbbb"<<std::endl;
        /// to-do 04 更新状态矩阵A并将状态矩阵A离散化
        UpdateMatrix(localization);

        // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
        // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;

        // to-do 05 Solve Lqr Problem
        SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_,
                        lqr_max_iteration_, &matrix_k_);

        // to-do 06 计算feedback
        //   feedback = - K * state
        //   Convert vehicle steer angle from rad to degree and then to steer degrees
        //   then to 100% ratio
        // std::cout << "KKKKKKKK" << this->matrix_k_ << std::endl;

        double steer_angle_feedback = -(this->matrix_k_ * this->matrix_state_)(0, 0);

        // to-do 07 计算前馈控制，计算横向转角的反馈量
        double steer_angle_feedforward = 0.0;
        // steer_angle_feedforward = ComputeFeedForward(localization, ref_curv_);
        double steer_angle = steer_angle_feedback - steer_angle_feedforward;
        //  std::cout << "steer_angle" << steer_angle << std::endl;
        // Set the steer commands
        if (steer_angle > 1.0)
            steer_angle = 1.0;
        else if (steer_angle < -1.0)
            steer_angle = -1.0;
        else
        {
        }
        cmd.steering_angle.data = 1.0 * steer_angle;

        return true;
    }
    void LQR_SOLVER::UpdateState(const VehicleState &vehicle_state,const Trajectory &traj)
    {
        // LateralControlError lat_con_err;  // 将其更改为智能指针
        std::shared_ptr<LateralControlError> lat_con_err =
            std::make_shared<LateralControlError>();
        ComputeLateralErrors(vehicle_state.x, vehicle_state.y, vehicle_state.yaw,
                             vehicle_state.vx, vehicle_state.w,
                             vehicle_state.a, lat_con_err,traj);

        // State matrix update;
        matrix_state_(0, 0) = lat_con_err->lateral_error;
        matrix_state_(1, 0) = lat_con_err->lateral_error_rate;
        matrix_state_(2, 0) = lat_con_err->heading_error;
        matrix_state_(3, 0) = lat_con_err->heading_error_rate;
    }
    void LQR_SOLVER::UpdateMatrix(const VehicleState &vehicle_state)
    {

        Matrix I = Matrix::Identity(this->matrix_a_.cols(), this->matrix_a_.rows());
        this->matrix_ad_ = (I - this->matrix_a_ * 0.5 * this->ts_).inverse() * (I + this->matrix_a_ * 0.5 * this->ts_);
    }
    void LQR_SOLVER::ComputeLateralErrors(const double x, const double y, const double theta,
                                          const double linear_v, const double angular_v,
                                          const double linear_a,
                                          LateralControlErrorPtr &lat_con_err,
                                          const Trajectory &traj)
    {

        // auto trajectory_points = this->QueryNearestPointByPosition(x, y);
        auto trajectory_points = traj[2];
        //      for(int i=0;i<trajectory_.size()-1;++i)
        // {
        //     std::cout<<i<<"ge x"<<trajectory_[i].pts.x<<std::endl;
        //     std::cout<<i<<"ge y"<<trajectory_[i].pts.y<<std::endl;
        //      std::cout<<i<<"ge yaw"<<trajectory_[i].yaw<<std::endl;
        // }
        //  std::cout << "planning point x" << trajectory_points.pts.x << std::endl;
        //  std::cout << "planning point y" << trajectory_points.pts.y << std::endl;
        // std::cout<< "planning point yaw"<<trajectory_points.yaw<<std::endl;
        //  std::cout << "state x" << x << std::endl;
        //  std::cout << "state y" << y << std::endl;
        double dx = -(trajectory_points.pts.x - x);
        double dy = -(trajectory_points.pts.y - y);

        // std::cout << "参考点角度:" << trajectory_points.yaw << std::endl;
        //  std::cout << "v:" << state_.v << std::endl;
        //  std::cout << "vx:" << state_.vx << std::endl;
        double e_y = dy * cos(trajectory_points.yaw) - dx * sin(trajectory_points.yaw);
        // double e_y=trajectory_points.pts.y*cos(trajectory_points.yaw)-trajectory_points.pts.x*sin(trajectory_points.yaw);
        double e_theta = -trajectory_points.yaw + theta;
        // double e_theta=trajectory_points.yaw;
        if (e_theta > M_PI)
        {
            e_theta = e_theta - M_PI * 2;
        }
        if (e_theta < -M_PI)
        {
            e_theta = e_theta + M_PI * 2;
        }

        // std::cout << "横向误差：" << e_y << std::endl;
        lat_con_err->lateral_error = e_y;
        lat_con_err->heading_error = e_theta;
        lat_con_err->lateral_error_rate = linear_v * std::sin(e_theta);
        lat_con_err->heading_error_rate = angular_v - trajectory_points.velocity * trajectory_points.curvature;
    }
    // ns_control::CsvPoint LQR_SOLVER::QueryNearestPointByPosition(const csvtraj trackingpath, const double x, const double y)
    // {

    //     double d_min = PointDistanceSquare(trackingpath.front(), x, y);
    //     size_t index_min = 0;

    //     for (size_t i = 1; i < trackingpath.size(); ++i)
    //     {
    //         double d_temp = PointDistanceSquare(trackingpath[i], x, y);
    //         if (d_temp < d_min)
    //         {
    //             d_min = d_temp;
    //             index_min = i;
    //         }
    //     }
    //     // cout << "x: " << trajectory_points_[index_min].x << " " << "y: " <<
    //     // trajectory_points_[index_min].y; cout << " index_min: " << index_min <<
    //     // endl; cout << "tarjectory.heading: " <<
    //     // trajectory_points_[index_min].heading << endl;

    //     // 对应的最近的轨迹点上的曲率

    //     return trackingpath[index_min];
    // }
    void LQR_SOLVER::SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                                     const Matrix &R, const double tolerance,
                                     const uint max_num_iteration, Matrix *ptr_K)
    {
        // 防止矩阵的维数出错导致后续的运算失败
        if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols())
        {
            std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
            return;
        }
        Matrix AT = A.transpose();
        Matrix BT = B.transpose();
        Matrix P = Q;
        double error = std::numeric_limits<double>::max();
        for (uint i = 0; i < max_num_iteration; ++i)
        {

            Matrix P_next = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
            error = fabs((-P + P_next).maxCoeff());
            P = P_next;
            if (error < tolerance)
            {
                std::cout << "keyifanhui" << error << std::endl;
                *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
                return;
            }
        }
    }

}