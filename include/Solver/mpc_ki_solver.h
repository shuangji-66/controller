
#pragma once
#include "Solver/solver_base.h"
#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"
#include <bits/stdc++.h>
#include <math.h>
#include "LinearMath/btMatrix3x3.h"
namespace ns_control {

class MPCKI_SOLVER : public Solver {
public:
  void solve();
  bool mpc_solve();
  protected:
      bool initMat(OsqpEigen::Solver& solver,Eigen::MatrixXd ref_state,Eigen::MatrixXd delta_state);
    bool temp(OsqpEigen::Solver& solver,Eigen::MatrixXd ref_state,Eigen::MatrixXd delta_state);
     Eigen::SparseMatrix<double> Hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double>  linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    void init();
    private:
    double ts=0.01;
    double wheelbase=1.88;

};

}; // namespace ns_control