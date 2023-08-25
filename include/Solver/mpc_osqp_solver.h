#ifndef __MPC_OSQP_SOLVER__
#define __MPC_OSQP_SOLVER__
#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include<iostream>
#include "Eigen/Eigen"
#include "osqp/osqp.h"

typedef Eigen::MatrixXd matrix;
using std::cout;
using std::endl;
// typedef struct {
//   c_int    n; ///< number of variables n
//   c_int    m; ///< number of constraints m
//   csc     *P; ///< the upper triangular part of the quadratic cost matrix P in csc format (size n x n).
//   csc     *A; ///< linear constraints matrix A in csc format (size m x n)
//   c_float *q; ///< dense array for linear part of cost function (size n)
//   c_float *l; ///< dense array for lower bound (size m)
//   c_float *u; ///< dense array for upper bound (size m)
// } OSQPData;

namespace ns_control{

class MPCosqp
{
public:
MPCosqp(const matrix &matrix_a,const matrix &matrix_b,const matrix &q,const matrix &r,
const matrix &initial_x,const matrix &u_lower,const matrix &u_upper,const matrix &x_lower
,const matrix & x_upper,const matrix &x_ref,const int max_iter,const int horizon ,const double eps_abs);

bool Solve(std::vector<double> *control_cmd);
private:
  void CalculateKernel(std::vector<c_float> *P_data,
                       std::vector<c_int> *P_indices,
                       std::vector<c_int> *P_indptr);
  void CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                   std::vector<c_int> *A_indices,
                                   std::vector<c_int> *A_indptr);
  void CalculateGradient();
  void CalculateConstraintVectors();
  OSQPSettings *Settings();
  OSQPData *Data();
  void FreeData(OSQPData *data);

  template <typename T>
  T *CopyData(const std::vector<T> &vec) {
    T *data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

 private:
  Eigen::MatrixXd matrix_a_;
  Eigen::MatrixXd matrix_b_;
  Eigen::MatrixXd matrix_q_;
  Eigen::MatrixXd matrix_r_;
  Eigen::MatrixXd matrix_initial_x_;
   Eigen::MatrixXd matrix_u_lower_;
   Eigen::MatrixXd matrix_u_upper_;
   Eigen::MatrixXd matrix_x_lower_;
   Eigen::MatrixXd matrix_x_upper_;
   Eigen::MatrixXd matrix_x_ref_;
  int max_iteration_;
  size_t horizon_;
  double eps_abs_;
  size_t state_dim_;
  size_t control_dim_;
  size_t num_param_;
  int num_constraint_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd lowerBound_;
  Eigen::VectorXd upperBound_;



};
}

#endif