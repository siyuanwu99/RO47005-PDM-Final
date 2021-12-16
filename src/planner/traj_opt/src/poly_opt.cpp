/**
 * @file poly_opt.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-05
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "traj_opt/poly_opt.h"

using namespace traj_utils;

/********** Polynomial Trajectory **********/
/**
 * @brief Polynomial trajectory constructor
 */
PolyTraj::PolyTraj(int n_pieces, int n_order) {
  num_ = n_pieces;
  order_ = n_order;
  x_pieces_.resize(num_);
  y_pieces_.resize(num_);
  z_pieces_.resize(num_);
  std::fill(x_pieces_.begin(), x_pieces_.end(),
            Eigen::VectorXd::Zero(order_ + 1));
  std::fill(y_pieces_.begin(), y_pieces_.end(),
            Eigen::VectorXd::Zero(order_ + 1));
  std::fill(z_pieces_.begin(), z_pieces_.end(),
            Eigen::VectorXd::Zero(order_ + 1));
  times_.resize(num_);
  std::fill(times_.begin(), times_.end(), 0.0);
}

/**
 * @brief get parameters of polynomial trajectory
 *
 * @param j
 * @param param
 */
void PolyTraj::getParams(int j, std::vector<Eigen::VectorXd>& param) {
  param.resize(N_DIM);
  param[0] = x_pieces_[j];
  param[1] = y_pieces_[j];
  param[2] = z_pieces_[j];
}

/**
 * @brief
 *
 * @param params [(order_ + 1) * num_]-by-[N_DIM] matrix
 */
void PolyTraj::setParams(const Eigen::MatrixXd& params) {
  for (auto i = 0; i < num_; i++) {
    x_pieces_[i] = params.block(i * (order_ + 1), 0, order_ + 1, 1);
    y_pieces_[i] = params.block(i * (order_ + 1), 1, order_ + 1, 1);
    z_pieces_[i] = params.block(i * (order_ + 1), 2, order_ + 1, 1);
  }
}

void PolyTraj::setTimeAllocation(const std::vector<double>& times) {
  times_ = times;
}

/**
 * @brief clear buffers which saves trajectory parameters
 *
 */
void PolyTraj::clearParams() {
  std::fill(x_pieces_.begin(), x_pieces_.end(),
            Eigen::VectorXd::Zero(order_ + 1));
  std::fill(y_pieces_.begin(), y_pieces_.end(),
            Eigen::VectorXd::Zero(order_ + 1));
  std::fill(z_pieces_.begin(), z_pieces_.end(),
            Eigen::VectorXd::Zero(order_ + 1));
  std::fill(times_.begin(), times_.end(), 0.0);
}

/**
 * @brief get positions
 *
 * @param t
 * @return Eigen::Vector3d
 */
Eigen::Vector3d PolyTraj::getWayPoints(double t) {
  int idx = num_;
  for (int i = 0; i < num_; i++) {
    if (t > times_[i]) {
      t -= times_[i];
    } else {
      idx = i;
      break;
    }
  }
  /* if t exceeds total time of trajectory */
  if (idx == num_) {
    idx = num_ - 1;
    t = times_[num_ - 1];
  }
  double x, y, z;
  x = x_pieces_[idx](0);
  y = y_pieces_[idx](0);
  z = z_pieces_[idx](0);

  for (auto j = 1; j < order_ + 1; j++) {
    x += x_pieces_[idx](j) * pow(t, j);
    y += y_pieces_[idx](j) * pow(t, j);
    z += z_pieces_[idx](j) * pow(t, j);
  }

  Eigen::Vector3d pos(x, y, z);
  // ROS_INFO_STREAM("waypoint pos\t" << idx << '\t' << t << '\t' << pos(0) <<
  // '\t' << pos(1)
  //                                  << '\t' << pos(2));
  return pos;
}

/**
 * @brief get velocities of PolynomialTrajectory
 *
 * @param t
 * @return Eigen::Vector3d
 */
Eigen::Vector3d PolyTraj::getVelocities(double t) {
  int idx = num_;
  for (int i = 0; i < num_; i++) {
    if (t > times_[i]) {
      t -= times_[i];
    } else {
      idx = i;
      break;
    }
  }
  /* if t exceeds total time of trajectory */
  if (idx == num_) {
    idx = num_ - 1;
    t = times_[num_ - 1];
  }
  double x, y, z;

  x = x_pieces_[idx](1);
  y = y_pieces_[idx](1);
  z = z_pieces_[idx](1);

  for (auto j = 2; j <= order_; j++) {
    x += j * pow(t, j - 1) * x_pieces_[idx](j);
    y += j * pow(t, j - 1) * y_pieces_[idx](j);
    z += j * pow(t, j - 1) * z_pieces_[idx](j);
  }

  Eigen::Vector3d vel(x, y, z);
  return vel;
}

/**
 * @brief get acclection of polynomial trajectory
 *
 * @param t
 * @return Eigen::Vector3d
 */
Eigen::Vector3d PolyTraj::getAcclections(double t) {
  int idx = num_;
  for (int i = 0; i < num_; i++) {
    if (t > times_[i]) {
      t -= times_[i];
    } else {
      idx = i;
      break;
    }
  }
  /* if t exceeds total time of trajectory */
  if (idx == num_) {
    idx = num_ - 1;
    t = times_[num_ - 1];
  }
  double x, y, z;

  x = 2 * x_pieces_[idx](2);
  y = 2 * y_pieces_[idx](2);
  z = 2 * z_pieces_[idx](2);
  for (auto j = 3; j <= order_; j++) {
    x += j * (j - 1) * pow(t, j - 2) * x_pieces_[idx](j);
    y += j * (j - 1) * pow(t, j - 2) * y_pieces_[idx](j);
    z += j * (j - 1) * pow(t, j - 2) * z_pieces_[idx](j);
  }

  Eigen::Vector3d acc(x, y, z);
  return acc;
}

// void PolyTraj::getWayPoints(double t)

/***************************  Mini Snap ************************************/

// MiniSnap::MiniSnap(const std::vector<Eigen::Vector3d>& pos,
//                    const std::vector<double>& times) {
//   order_ = 7;
//   n_variables_ = order_ + 1;
//   n_pieces_ = times.size();
//   T_ = times;
// }

// MiniSnap::MiniSnap(int num, const std::vector<Eigen::Vector3d>& pos,
//                    const std::vector<double>& times) {
//   order_ = 7;
//   n_variables_ = order_ + 1;
//   n_pieces_ = num;
//   if (n_pieces_ != times.size()) {
//     ROS_ERROR("Number of pieces mismatch.");
//   }
//   T_ = times
// }

// void MiniSnap::initParams() {
//   Q_.resize(n_pieces_);
//   std::fill(Q_.begin(), Q_.end(),
//             Eigen::MatrixXd::Zero(n_variables_, n_variables_));
//   T.resize(n_pieces_);
//   std::fill(T.begin(), T.end(), 0.0);
//   p.resize(n_pieces_);
//   std::fill(p.begin(), p.end(), Eigen::VectorXd::Zero(n_variables_));
// }

// void MiniSnap::calcCost(const std::vector<double>& x, std::vector<double>&
// grad,
//                         double& cost) {
//   cost = 0;
//   grad.resize(n_pieces_ * n_variables_);
//   std::fill(grad.begin(), grad.end(), 0.0);

//   for (auto k = 0; k < n_pieces_; k++) {
//     for (auto i = 0; i < n_variables_; i++) {
//       P_[k](i) = x[k * n_pieces_ + i];
//     }
//   }

//   for (auto k = 0; k < n_pieces_; k++) {
//     t = T_[k];
//     Eigen::VectorXd p = P_[k];
//     for (auto i = 4; i <= order_; i++) {
//       double gradient = 0.0;
//       for (auto j = 4; j <= order_; j++) {
//         coeff = i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) *
//                 (j - 3) / (i + j - 7) * pow(t, i + j - 7);
//         cost += coeff * p(i) * p(j);
//         gradient += coeff p(j);
//       }
//       grad[k * n_pieces_ + i] = gradient;
//     }
//   }
// }

// void MiniSnap::costFunction(const std::vector<double>& x,
//                             std::vector<double>& grad, void* func_data) {
//   MiniSnap* opt = reinterpret_cast<MiniSnap*>(func_data);
//   double cost;
//   opt->combineCost(x, grad, cost);
//   opt->opt_i_iterations_++;

//   return cost;
// }

// void MiniSnap::optimize() {
//   opt_i_iterations_ = 0;

//   /* init NLOPT solver */
//   nlopt::opt solver(nlopt::algorithm(NLOPT_LD_LBFGS), n_pieces_ *
//   n_variables_); solver.set_xtol_rel(1e-5);

//   /* optimization objective function */
//   solver.set_min_objective(MiniSnap::costFunction, this);

//   /* linear constraints */
//   std::vector<double> tol_constraints;
//   std::fill(tol_constraints.begin(), tol_constraints.end(), 1e-5);
//   solver.add_equality_mconstraint(MiniSnap::positionConstraints, this,
//   tol_constraints);

//   /* QP variables */
//   std::vector<double> x(n_pieces_ * n_variables_);
//   std::fill(x.begin(), x.end(), 0.0);

//   /* try to solve QP problem */
//   try {
//     double final_cost;
//     nlopt::result rst = solver.optimize(x, final_cost);
//   } catch (std::exception& e) {
//     ROS_WARN("[Optimization]: nlopt exception: %s", e.what());
//   }
// }

/********** MiniSnap Closed Form **********/
/**
 * @brief constructor
 */
MiniSnapClosedForm::MiniSnapClosedForm(const std::vector<Eigen::Vector3d>& pos,
                                       const std::vector<double>& times) {
  order_ = 7;
  n_variables_ = order_ + 1;
  n_pieces_ = times.size();
  Pos_ = pos;
  T_ = times;
}

void MiniSnapClosedForm::initParams() {
  // int d_order_ = 4;
  p_length_ = n_variables_ * n_pieces_;
  d_order_ = n_variables_ / 2;
  d_length_ = (n_pieces_ + 1) * d_order_;
  Q_ = Eigen::MatrixXd::Zero(p_length_, p_length_);
  M_ = Eigen::MatrixXd::Zero(p_length_, p_length_);
  C_ = Eigen::MatrixXd::Zero(p_length_, d_order_ * (n_pieces_ + 1));
  P_ = Eigen::MatrixXd::Zero(p_length_, N_DIM);
  ROS_INFO_STREAM("[MiniSnap] number of pieces\t\t" << n_pieces_);
  ROS_INFO_STREAM("[MiniSnap] opt variables length\t\t" << p_length_);
}

void MiniSnapClosedForm::getQ() {
  double t;
  for (auto k = 0; k < n_pieces_; k++) {
    t = T_[k];
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(n_variables_, n_variables_);
    for (auto i = 4; i < n_variables_; i++) {
      for (auto j = 4; j < n_variables_; j++) {
        Q_k(i, j) = i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) *
                    (j - 3) / (i + j - 7) * pow(t, i + j - 7);
      }
    }
    // std::cout << "Q_k\n " << Q_k << std::endl;
    Q_.block(k * n_variables_, k * n_variables_, n_variables_, n_variables_) =
        Q_k;
  }
}

void MiniSnapClosedForm::getM() {
  double t;

  for (auto k = 0; k < n_pieces_; k++) {
    t = T_[k];
    Eigen::MatrixXd M_k = Eigen::MatrixXd::Zero(n_variables_, n_variables_);
    M_k << 1, 0, 0, 0, 0, 0, 0, 0,  // 1st row
        0, 1, 0, 0, 0, 0, 0, 0,     // 2nd row
        0, 0, 2, 0, 0, 0, 0, 0,     // 3rd row
        0, 0, 0, 6, 0, 0, 0, 0,     // 4th row
        1, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6),
        pow(t, 7),  // 5th row
        0, 1, 2 * pow(t, 1), 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
        6 * pow(t, 5), 7 * pow(t, 6),  // 6th row
        0, 0, 2, 6 * pow(t, 1), 12 * pow(t, 2), 20 * pow(t, 3), 30 * pow(t, 4),
        42 * pow(t, 5),  // 7th row
        0, 0, 0, 6, 24 * pow(t, 1), 60 * pow(t, 2), 120 * pow(t, 3),
        210 * pow(t, 4);  // final row
    M_.block(k * n_variables_, k * n_variables_, n_variables_, n_variables_) =
        M_k;
    // std::cout << "M_ \n" << M_k << std::endl;
  }
}

void MiniSnapClosedForm::getCt() {
  C_.block(0, 0, d_order_, d_order_) =
      Eigen::MatrixXd::Identity(d_order_, d_order_);
  C_.block((n_pieces_ - 1) * n_variables_ + d_order_,
           d_order_ + (n_pieces_ - 1), d_order_, d_order_) =
      Eigen::MatrixXd::Identity(d_order_, d_order_);

  for (auto k = 0; k < n_pieces_ - 1; k++) {
    Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(n_variables_, d_length_);
    Ct(0, d_order_ + k) = 1;
    Ct(1, d_order_ + d_order_ + (n_pieces_ - 1) + 3 * k) = 1;
    Ct(2, d_order_ + d_order_ + (n_pieces_ - 1) + 3 * k + 1) = 1;
    Ct(3, d_order_ + d_order_ + (n_pieces_ - 1) + 3 * k + 2) = 1;
    Ct(4, d_order_ + k) = 1;
    Ct(5, d_order_ + d_order_ + (n_pieces_ - 1) + 3 * k) = 1;
    Ct(6, d_order_ + d_order_ + (n_pieces_ - 1) + 3 * k + 1) = 1;
    Ct(7, d_order_ + d_order_ + (n_pieces_ - 1) + 3 * k + 2) = 1;

    C_.block(d_order_ + n_variables_ * k, 0, n_variables_, d_length_) = Ct;
    // std::cout << "C_t\n " << Ct << std::endl;
  }
}

/**
 * @brief solve Mini Snap problem for single dimension
 * @param i_dim
 */
void MiniSnapClosedForm::solveSingle(int i_dim) {
  int dF_length = 2 * d_order_ + n_pieces_ - 1;
  int dP_length = (d_order_ - 1) * (n_pieces_ - 1);

  getM();
  getQ();
  getCt();

  Eigen::MatrixXd M_inv = M_.inverse();
  Eigen::MatrixXd M_inv_trans = M_inv.transpose();

  Eigen::MatrixXd R = C_.transpose() * M_inv_trans * Q_ * M_inv * C_;
  Eigen::MatrixXd R_pp = R.block(dF_length, dF_length, dP_length, dP_length);
  Eigen::MatrixXd R_fp = R.block(0, dF_length, dF_length, dP_length);

  // get dF
  Eigen::VectorXd dF(dF_length);
  Eigen::Vector4d temp;
  temp << Pos_[0][i_dim], 0.0, 0.0, 0.0;
  dF.head(d_order_) = temp;
  for (auto k = 1; k < n_pieces_; k++) {
    dF(d_order_ + k - 1) = Pos_[k][i_dim];
  }
  temp << Pos_[n_pieces_][i_dim], 0.0, 0.0, 0.0;
  dF.tail(d_order_) = temp;

  // std::cout << "dF\n" << dF << std::endl;

  // get dP
  Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;

  Eigen::VectorXd dF_cat_dP(dF_length + dP_length);
  dF_cat_dP << dF, dP;

  // save results to P_
  P_.block(0, i_dim, p_length_, 1) = M_inv * C_ * dF_cat_dP;
}

void MiniSnapClosedForm::solve(PolyTraj* traj) {
  for (auto d = 0; d < N_DIM; d++) {
    solveSingle(d);
  }
  ROS_INFO("Successfully got Mini Snap Closed-form solution");
  traj->setParams(P_);
  traj->setTimeAllocation(T_);
}