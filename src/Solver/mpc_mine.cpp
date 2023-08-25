#include "Solver/mpc_mine.h"
#include "ros/ros.h"
#include <cmath>
#include "Solver/pid_controller.h"
namespace ns_control
{

  void MPC_MINE_Solver::solve()
  {
    ROS_INFO_STREAM("begin solve");
    if (trajectory_.empty())
    {
      control_command_.throttle.data = static_cast<float>(-1.0); // 类型转换
      control_command_.steering_angle.data = 0.0;
      ROS_INFO_STREAM("trajectory empty");
      return;
    }
    LoadControlConf();
    init();
    double desire_vel = control_param_.desire_vel;
    if (!ComputeControlCommand(
            state_,
            trajectory_, control_command_))
      std::cout << "failed" << std::endl;

    // { // Speed Controller
    //     const double vel = state_.v;
    //     control_command_.throttle.data = static_cast<float>(desire_vel - vel);
    // }
  }
  void MPC_MINE_Solver::LoadControlConf()
  {
    ts_ = 0.01;                               // 每隔0.01s进行一次控制
    cf_ = 88539.01;                           // 前轮侧偏刚度,左右轮之和
    cr_ = 88539.01;                           // 后轮侧偏刚度, 左右轮之和
    wheelbase_ = 1.88;                        // 左右轮的距离
    steer_ratio_ = 10;                        // 方向盘的转角到轮胎转动角度之间的比值系数
    steer_single_direction_max_degree_ =200; // 最大方向转角

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

    mpc_eps_ = 0.01;           // MPC迭代求解精度
    mpc_max_iteration_ = 1500; // MPC的迭代次数
    double wheel_single_direction_max_degree_ =
        steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;

    return;
  }

  // 初始化控制器
  void MPC_MINE_Solver::init()
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
    // matrix_a_(5, 5) = 1.0;
    //  初始化A矩阵的非常数项
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
     //matrix_b_(5, 1) = -1;
    matrix_bd_ = matrix_b_ * ts_;
    matrix_c_ = Matrix::Zero(basic_state_size_, 1);
    matrix_cd_ = Matrix::Zero(basic_state_size_, 1);
    // 状态向量
    matrix_state_ = Matrix::Zero(matrix_size, 1);
    // 反馈矩阵
    matrix_k_ = Matrix::Zero(1, matrix_size);
    // lqr cost function中 输入值u的权重
    matrix_r_ = Matrix::Identity(1, 1);
    // matrix_r_(0, 0) = 100;
    //  matrix_r_(1, 0) = 100;
    //  matrix_r_(0, 1) = 0;
    //  matrix_r_(1, 1) = 100;
    //  lqr cost function中 状态向量x的权重
    matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

     int q_param_size = 6;
    matrix_q_(0, 0) = 100; // lateral_error
    matrix_q_(1, 1) = 100; // lateral_error_rate
    matrix_q_(2, 2) = 100; // heading_error
    matrix_q_(3, 3) = 100; // heading__error_rate
     matrix_q_(4, 4) = 100; // station_error
     matrix_q_(5, 5) = 100; // station_rate_error

    matrix_q_updated_ = matrix_q_;
    matrix_r_updated_ = matrix_r_;

    return;
  }

  // **to-do**计算控制命令
  bool MPC_MINE_Solver::ComputeControlCommand(
      const VehicleState &localization,
      const Trajectory &planning_published_trajectory, fsd_common_msgs::ControlCommand &cmd)
  {
    // 规划轨迹
    auto trajectory_points_ = planning_published_trajectory[2];

    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
    0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
    (l_r * c_r - l_f * c_f) / m / v;
    0.0, 0.0, 0.0, 1.0;
    0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
    (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */

    // to-do 01 配置状态矩阵A
    // this->matrix_a_(0, 1) = 1.0;
    // this->matrix_a_coeff_(1, 1) = -(this->cf_ + this->cr_) / this->mass_;
    // this->matrix_a_(1, 2) = (this->cf_ + this->cr_) / this->mass_;
    // this->matrix_a_coeff_(1, 3) = (this->lr_ * this->cr_ - this->lf_ * this->cf_) / this->mass_;
    // this->matrix_a_(2, 3) = 1.0;
    // this->matrix_a_coeff_(3, 1) = (this->lr_ * this->cr_ - this->lf_ * this->cf_) / this->iz_;
    // this->matrix_a_coeff_(3, 3) = -1 * (this->lf_ * this->lf_ * this->cf_ + this->lr_ * this->lr_ * this->cr_) / this->iz_;
    // this->matrix_a_(3, 2) = (this->lf_ * this->cf_ - this->lr_ * this->cr_) / this->iz_;

    double v_ = localization.v > this->minimum_speed_protection_ ? localization.v : this->minimum_speed_protection_;
    this->matrix_a_(1, 1) = this->matrix_a_coeff_(1, 1) / v_;
    this->matrix_a_(1, 3) = this->matrix_a_coeff_(1, 3) / v_;
    this->matrix_a_(3, 1) = this->matrix_a_coeff_(3, 1) / v_;
    this->matrix_a_(3, 3) = this->matrix_a_coeff_(3, 3) / v_;
    double wheel_single_direction_max_degree_ =
        steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;
    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    // to-do 02 动力矩阵B

    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    this->matrix_bd_ = this->matrix_b_ * this->ts_;
    // //
    //    cout<< "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
    // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;
    //  Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
    //  Error Rate]

    // to-do 03 计算横向误差并且更新状态向量x
    UpdateState(localization);

    /// to-do 04 更新状态矩阵A并将状态矩阵A离散化
    UpdateMatrix(localization);

    Matrix control_matrix = Matrix::Zero(this->control_, 1);
    std::vector<Matrix> control_m(this->horizen_, control_matrix);

    Matrix reference_state = Matrix::Zero(basic_state_size_, 1);
    std::vector<Matrix> reference(horizen_, reference_state);

    // Matrix lower_bound(control_, 1);
    // lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;
    Matrix lower_bound(control_, 1);
    lower_bound << -wheel_single_direction_max_degree_;
    // Matrix upper_bound(control_, 1);
    // upper_bound << wheel_single_direction_max_degree_, max_acceleration_;
    Matrix upper_bound(control_, 1);
    upper_bound << wheel_single_direction_max_degree_;
    const double max = std::numeric_limits<double>::max();
    Matrix lower_state_bound(basic_state_size_, 1);
    Matrix upper_state_bound(basic_state_size_, 1);
    // double mpc_start_timestamp = Clock::NowInSeconds();
    //  lateral_error, lateral_error_rate, heading_error, heading_error_rate
    //  station_error, station_error_rate
    // lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI, -1.0 * max,
    //     -1.0 * max, -1.0 * max;
    // upper_state_bound << max, max, M_PI, max, max, max;
    lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * max, -1.0 * max;
    upper_state_bound << max, max, max, max;
    std::vector<double> control_cmd(1, 0);
    MPCosqp mpc_solve(matrix_ad_, matrix_bd_, matrix_q_updated_, matrix_r_updated_,
                      matrix_state_, lower_bound, upper_bound, lower_state_bound,
                      upper_state_bound, reference_state, mpc_max_iteration_, horizen_,
                      mpc_eps_);
    if (!mpc_solve.Solve(&control_cmd))
      std::cout << "MPC solve failed" << std::endl;
    else
    {
      std::cout << "MPC sovle succeed" << std::endl;
      control_m[0](0, 0) = control_cmd.at(0);

      // control_m[0](1, 0) = control_cmd.at(1);
    }

    double steer_angle_feedback = control_m[0](0, 0);

    // to-do 07 计算前馈控制，计算横向转角的反馈量
    double steer_angle_feedforward = 0.0;
    // steer_angle_feedforward = ComputeFeedForward(localization, ref_curv_);
    steer_angle_feedforward = 0.0;
    double steer_angle = steer_angle_feedback - steer_angle_feedforward;
    // cout << "steer_angle" << steer_angle << endl;
    //  Set the steer commands
    PIDController speedpid(0.001, 0.1, 0);
    cmd.steering_angle.data = steer_angle;
    // cmd.throttle.data = 2 * (trajectory_[3].acc + control_m[0](1, 0));
    cmd.throttle.data = speedpid.Control(trajectory_points_.velocity - state_.v, 0.1);
    return true;
  }

  // 计算横向误差并且更新状态向量x
  void MPC_MINE_Solver::UpdateState(const VehicleState &vehicle_state)
  {
    // LateralControlError lat_con_err;  // 将其更改为智能指针
    std::shared_ptr<LateralControlError> lat_con_err =
        std::make_shared<LateralControlError>();
    std::shared_ptr<LongitudinalError> long_err = std::make_shared<LongitudinalError>();
    // 计算横向误差
    ComputeLateralErrors(vehicle_state.x, vehicle_state.y, vehicle_state.yaw,
                         vehicle_state.v, vehicle_state.w,
                         vehicle_state.a, lat_con_err);
    // ComputeLongitudinalErrors(vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw, long_err);

    // State matrix update;
    matrix_state_(0, 0) = lat_con_err->lateral_error;
    matrix_state_(1, 0) = lat_con_err->lateral_error_rate;
    matrix_state_(2, 0) = lat_con_err->heading_error;
    matrix_state_(3, 0) = lat_con_err->heading_error_rate;
    // matrix_state_(4, 0) = long_err->station_error;
    // matrix_state_(5, 0) = long_err->station_rate_error;
    //  cout << "lateral_error: " << (lat_con_err->lateral_error) << endl;
    //  cout << "heading_error: " << (lat_con_err->heading_error) << endl;
  }

  // to-do 04 更新状态矩阵A并将状态矩阵A离散化
  void MPC_MINE_Solver::UpdateMatrix(const VehicleState &vehicle_state)
  {

    Matrix I = Matrix::Identity(this->matrix_a_.cols(), this->matrix_a_.rows());
    this->matrix_ad_ = (I - this->matrix_a_ * 0.5 * this->ts_).inverse() * (I + this->matrix_a_ * 0.5 * this->ts_);
  }

  // to-do 03 计算hengxiang误差
  void MPC_MINE_Solver::ComputeLateralErrors(const double x, const double y, const double theta, const double linear_v, const double angular_v, const double linear_a, LateralControlErrorPtr &lat_con_err)
  {

    // auto trajectory_points = this->QueryNearestPointByPosition(x, y);
    auto trajectory_points = trajectory_[2];
    //       for(int i=0;i<trajectory_.size()-1;++i)
    // {
    //      std::cout<<i<<"ge x"<<trajectory_[i].pts.x<<std::endl;
    //     std::cout<<i<<"ge y"<<trajectory_[i].pts.y<<std::endl;
    //       std::cout<<i<<"ge yaw"<<trajectory_[i].yaw<<std::endl;
    //  }
    // std::cout << "planning point x" << trajectory_points.pts.x << std::endl;
    // std::cout << "planning point y" << trajectory_points.pts.y << std::endl;
    // std::cout << "planning point yaw" << trajectory_points.yaw << std::endl;
    // std::cout << "state x" << x << std::endl;
    // std::cout << "state y" << y << std::endl;
    double dx = trajectory_points.pts.x ;
    double dy = trajectory_points.pts.y ;

    // std::cout << "参考点角度:" << trajectory_points.yaw << std::endl;
    std::cout << "主车角度:" << theta << std::endl;
    std::cout << "state角度:" << state_.yaw << std::endl;
double planning_yaw=trajectory_points.yaw;


    double e_y = dy * cos(planning_yaw) - dx * sin(planning_yaw);

    double e_theta = -trajectory_points.yaw +theta;
    if (e_theta > M_PI)
    {
      e_theta = e_theta - M_PI * 2;
    }
    if (e_theta < -M_PI)
    {
      e_theta = e_theta + M_PI * 2;
    }

    std::cout << "横向误差：" << e_y << std::endl;
      std::cout << "航向误差：" << e_theta << std::endl;
    lat_con_err->lateral_error = e_y;
    lat_con_err->heading_error = e_theta;
    lat_con_err->lateral_error_rate = linear_v * std::sin(e_theta);
    lat_con_err->heading_error_rate = angular_v - trajectory_points.velocity * trajectory_points.curvature;
  }

  void MPC_MINE_Solver::ComputeLongitudinalErrors(const double x, const double y, const double v, const double theta, LongitudinalErrorPtr &lon_err)
  {
    auto trajectory_points = trajectory_[3];
    double dx = trajectory_points.pts.x;
    double dy = trajectory_points.pts.y;
    double es = (dx * cos(trajectory_points.yaw) + dy * sin(trajectory_points.yaw));
    double delta_theta = theta - trajectory_points.yaw;
    if (delta_theta > M_PI)
    {
      delta_theta = delta_theta - M_PI * 2;
    }
    if (delta_theta < -M_PI)
    {
      delta_theta = delta_theta + M_PI * 2;
    }
    double cos_delta_theta = std::cos(delta_theta);
    double sin_delta_theta = std::sin(delta_theta);
    double cross_rd_nd = cos(trajectory_points.yaw) * dy - sin(trajectory_points.yaw) * dx;
    double one_minus_kappa_r_d = 1 - trajectory_points.yaw * cross_rd_nd;
    if (one_minus_kappa_r_d <= 0.0)
      one_minus_kappa_r_d = 0.01;
    // double es_dot = trajectory_points.velocity - v * cos_delta_theta / one_minus_kappa_r_d;
    double es_dot = 5 - v * cos_delta_theta / one_minus_kappa_r_d;
    lon_err->station_error = es;
    lon_err->station_rate_error = es_dot;
  }
  // to-do 05:求解MPC

}