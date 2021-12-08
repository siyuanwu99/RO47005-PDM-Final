/**
 * @file poly_opt.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-05
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef _POLY_OPT_H
#define _POLY_OPT_H
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <traj_opt/lbfgs.hpp>
// #include <nlopt/nlopt.hpp>
#include <vector>

namespace traj_utils {
#define N_DIM 3  // number of dimensions

/**
 * @brief save polynomial trajectory parameters
 *
 */
class PolyTraj {
 private:
  int num_;    // number of pieces
  int order_;  // order of polynomial
  std::vector<Eigen::VectorXd> x_pieces_;
  std::vector<Eigen::VectorXd> y_pieces_;
  std::vector<Eigen::VectorXd> z_pieces_;
  std::vector<double> times_;

 public:
  PolyTraj() {}
  ~PolyTraj() {}
  PolyTraj(int n_pieces, int n_order);
  void getParams(int j, std::vector<Eigen::VectorXd>& param);
  void setParams(const Eigen::MatrixXd& params);
  void setTimeAllocation(const std::vector<double>& times);
  Eigen::Vector3d getWayPoints(double t);
  Eigen::Vector3d getVelocities(double t);
  Eigen::Vector3d getAcclections(double t);
  void clearParams();
};

// class MiniSnap {
//  private:
//   int order_;
//   int n_pieces_;
//   int n_variables_;
//   Eigen::Matrix3d head_p_v_a_;
//   Eigen::Matrix3d tail_p_v_a_;
//   // Eigen::VectorXd T_;
//   Eigen::MatrixXd A_;
//   std::vector<Eigen::MatrixXd> Q_;

//   /** @brief time allocation for each pieces of polynomial trajectory */
//   std::vector<double> T_;

//   /** @brief parameters for each pieces of polynomial trajectory */
//   std::vector<Eigen::VectorXd> P_;

//   // PolyTraj traj_;

//   /* variables for optimization problem solving */
//   int opt_i_iterations_;

//  public:
//   MiniSnap() { order_ = 7; }
//   MiniSnap(int num, const std::vector<Eigen::Vector3d>& pos,
//            const std::vector<double>& times);
//   MiniSnap(const std::vector<Eigen::Vector3d>& pos,
//            const std::vector<double>& times);
//   ~MiniSnap() {}
//   void initParams();
//   void optimize();
//   void getParams();

//   void continuityConstraints(unsigned m, double* result, unsigned n,
//                              const double* x, double* grad, void* f_data);
//   void calcContinuityConstraints();
//   void positionConstraints(unsigned m, double* result, unsigned n,
//                            const double* x, double* grad, void* f_data);
//   void calcPositionConstraints();

//   void calcCost(const std::vector<double>& x, std::vector<double>& grad,
//                 double& cost);
//   void costFunction(const std::vector<double>& x, std::vector<double>& grad,
//                     void* func_data)

//       typedef std::unique_ptr<MiniSnap> Ptr;

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };

class MiniSnapClosedForm {
 private:
  int order_;
  int n_pieces_;
  int n_variables_;
  int p_length_;
  int d_length_;
  int d_order_;

  Eigen::MatrixXd Q_;  // Hessian
  Eigen::MatrixXd M_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd P_;  // polynomial parameters

  std::vector<double> T_;
  std::vector<Eigen::Vector3d> Pos_;

 public:
  MiniSnapClosedForm() {
    order_ = 7;
    n_variables_ = order_ + 1;
  }

  MiniSnapClosedForm(const std::vector<Eigen::Vector3d>& pos,
                     const std::vector<double>& times);

  ~MiniSnapClosedForm() {}

  void initParams();
  void solve(PolyTraj* traj);
  void solveSingle(int i_dim);
  void getM();
  void getCt();
  void getQ();
};

}  // namespace traj_utils

#endif /* _POLY_OPT_H */
