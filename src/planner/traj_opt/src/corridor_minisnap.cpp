/**
 * @file corridor_minisnap.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <traj_opt/corridor_minisnap.h>

#include <traj_opt/root_finder.hpp>

using namespace traj_opt;

/********************************/
/********** Poly Piece **********/
/********************************/

void PolyPiece::setup(const double t) { _duration = t; }
void PolyPiece::setup(const Eigen::MatrixXd coeffs) { _coeffs = coeffs; }
void PolyPiece::setup(const double t, const Eigen::MatrixXd coeffs) {
  setup(t);
  setup(coeffs);
}

/**
 * @brief get position
 * @param t relative time stamp
 * @return Eigen::Vector3d
 */
Eigen::Vector3d PolyPiece::getPos(double t) const {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::VectorXd T(N_ORDER + 1);
  T(0) = 1;
  for (int i = 1; i <= N_ORDER; i++) {
    T(i) = pow(t, i);
  }
  pos = _coeffs * T;
  // std::cout << t << '\t' << pos.transpose() << std::endl;
  return pos;
}

Eigen::Vector3d PolyPiece::getVel(double t) const {
  Eigen::VectorXd T(N_ORDER);
  T(0) = 1;
  for (int i = 1; i <= N_ORDER - 1; i++) {
    T(i) = pow(t, i) * (i + 1);
  }
  Eigen::Vector3d vel = _coeffs.block<3, 7>(0, 1) * T;
  return vel;
}

Eigen::Vector3d PolyPiece::getAcc(double t) const {
  Eigen::VectorXd T(N_ORDER - 1);
  T(0) = 2;
  for (int i = 1; i <= N_ORDER - 2; i++) {
    T(i) = pow(t, i) * (i + 1) * (i + 2);
  }
  Eigen::Vector3d acc = _coeffs.block<3, 6>(0, 2) * T;
  return acc;
}

Eigen::Matrix3Xd Trajectory::getPositions() const {
  int N = getPieceNum();
  Eigen::Matrix3Xd positions(3, N + 1);
  for (int i = 0; i < N; i++) {
    positions.col(i) = _pieces[i].getCoefficient().col(0);
  }
  positions.col(N) = _pieces[N - 1].getPos(1);
  return positions;
}

double PolyPiece::getDuration() const { return _duration; }

Eigen::Matrix<double, DIM, N_ORDER + 1> PolyPiece::getCoefficient() const {
  return _coeffs;
}

/********************************/
/********** Trajectory **********/
/********************************/

void Trajectory::setDuration(const std::vector<double> &t) {
  N = t.size();
  _pieces.resize(N);
  for (int i = 0; i < N; i++) {
    _pieces[i].setup(t[i]);
  }
}

void Trajectory::setCoefficient(const Eigen::VectorXd &x) {
  int M = N_ORDER + 1;
  for (int i = 0; i < N; i++) {
    Eigen::Matrix<double, DIM, N_ORDER + 1> temp;
    temp << x.segment(DIM * M * i, M).transpose(),
        x.segment(DIM * M * i + M, M).transpose(),
        x.segment(DIM * M * i + 2 * M, M).transpose();
    _pieces[i].setup(temp);
  }
}

/**
 * @brief given absolute timestamp, find out piece index and relative timestamp
 * @param t0 absolute time stamp
 * @param t  return t \in [0, 1)
 * @param idx
 */
inline void Trajectory::locatePiece(const double &t0, double &t,
                                    int &idx) const {
  idx = N;
  double tmp = t0;
  for (int i = 0; i < N; i++) {
    double Ti = _pieces[i].getDuration();
    if (tmp > Ti) {
      tmp -= Ti;
    } else {
      idx = i;
      t = tmp / Ti;
      break;
    }
  }

  /* if t0 is longer than all durations */
  if (idx == N) {
    idx = N - 1;
    t = 1;
  }
}

/**
 * @brief
 * @param t absolute time stamp
 * @return Eigen::Vector3d
 */
Eigen::Vector3d Trajectory::getPos(double t) const {
  double relative_time;
  int index;
  locatePiece(t, relative_time, index);
  // std::cout << "idx " << index << "\t";
  return _pieces[index].getPos(relative_time);
}

Eigen::Vector3d Trajectory::getVel(double t) const {
  double relative_time;
  int index;
  locatePiece(t, relative_time, index);
  return _pieces[index].getVel(relative_time);
}

Eigen::Vector3d Trajectory::getAcc(double t) const {
  double relative_time;
  int index;
  locatePiece(t, relative_time, index);
  return _pieces[index].getAcc(relative_time);
}

double Trajectory::getDuration() const {
  double T = 0;
  for (size_t i = 0; i < N; i++) {
    T += _pieces[i].getDuration();
  }
  return T;
}

double Trajectory::getMaxVelRate() const {
  double T = 0;
  double V = 0;
  for (double t = 0; t < getDuration(); t += 0.1) {
    Eigen::Vector3d v;
    v = this->getVel(t);
    double v_rate = v.norm();
    if (v_rate > V) {
      V = v_rate;
    }
  }
  return V;
}

double Trajectory::getMaxAccRate() const {
  double T = 0;
  double A = 0;
  for (double t = 0; t < getDuration(); t += 0.1) {
    Eigen::Vector3d a;
    a = this->getAcc(t);
    double a_rate = a.norm();
    if (a_rate > A) {
      A = a_rate;
    }
  }
  return A;
}


int Trajectory::getPieceNum() const { return N; }

/**********************************/
/********** Minimun Snap **********/
/**********************************/

void MiniSnap::reset(const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
                     const std::vector<Eigen::Vector3d> &waypoints,
                     const std::vector<double> &timeAlloc) {
  ROS_INFO_STREAM("[TrajOpt] head state:\n" << head);
  ROS_INFO_STREAM("[TrajOpt] tail state:\n" << tail);
  _headPVA = head;
  _tailPVA = tail;
  ROS_INFO_STREAM("[TrajOpt] waypoints:" << waypoints.size());
  ROS_INFO_STREAM("[TrajOpt] timeAlloc:" << timeAlloc.size());
  _waypoints = waypoints;
  _timeAlloc = timeAlloc;
  N = timeAlloc.size();
  int S = N * (N_ORDER + 1) * DIM;  // number of variables
  _x.resize(S);
  _Q.resize(S, S);
  _Q.setZero();
  int M = 15 * N + 9;  // number of all constraints
  _A.resize(M, S);
  _A.setZero();
  _ub.resize(M);
  _ub.setZero();
  _lb.resize(M);
  _lb.setZero();
}

/* T = 1 */
void MiniSnap::getCostFunc() {
  /* for single piece, single dimension */
  int D = N_ORDER + 1;  // size of matrix Q
  Eigen::Matrix<double, N_ORDER + 1, N_ORDER + 1> Q;
  for (int i = 0; i <= N_ORDER; i++) {
    for (int j = 0; j <= N_ORDER; j++) {
      if (i < 4 || j < 4) {
        Q(i, j) = 0;
      }
      if (i + j > N_ORDER) {
        Q(i, j) = i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) *
                  (j - 3) / (i + j - N_ORDER);
      }
    }
  }
  /* iterate all dimensions and all pieces */
  for (int i = 0; i < N * DIM; i++) {
    _Q.block(i * D, i * D, D, D) = Q;
  }
}

/**
 * @brief waypoints contraints: equality constraints for passing through fixed
 * waypoints
 */
void MiniSnap::getHeadTailConstraint() {
  /* constraints for starting states*/
  for (int i = 0; i < DIM; i++) {
    _A(0 + 4 * i, 0 + 8 * i) = 1;
    _A(1 + 4 * i, 1 + 8 * i) = 1;
    _A(2 + 4 * i, 2 + 8 * i) = 2;
    _A(3 + 4 * i, 3 + 8 * i) = 6;
    _ub(0 + 4 * i) = _headPVA(i, 0);
    _ub(1 + 4 * i) = _headPVA(i, 1);
    _ub(2 + 4 * i) = _headPVA(i, 2);
    _ub(3 + 4 * i) = 0;
    _lb(0 + 4 * i) = _headPVA(i, 0);
    _lb(1 + 4 * i) = _headPVA(i, 1);
    _lb(2 + 4 * i) = _headPVA(i, 2);
    _lb(3 + 4 * i) = 0;
  }

  /* constraints for end states */
  int M = (N - 1) * (N_ORDER + 1) * DIM;
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;
  for (int i = 0; i < DIM; i++) {
    _A.block(12 + 0 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = pos_1d;
    _A.block(12 + 1 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = vel_1d;
    _A.block(12 + 2 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = acc_1d;
    _A.block(12 + 3 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = jer_1d;
    _ub(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _ub(12 + 1 + 4 * i) = _tailPVA(i, 1);
    _ub(12 + 2 + 4 * i) = _tailPVA(i, 2);
    _ub(12 + 3 + 4 * i) = 0;
    _lb(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _lb(12 + 1 + 4 * i) = _tailPVA(i, 1);
    _lb(12 + 2 + 4 * i) = _tailPVA(i, 2);
    _lb(12 + 3 + 4 * i) = 0;
  }
}

void MiniSnap::getWaypointsConstraint() {
  /* constraints for intermediate states*/
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  for (int j = 0; j < N - 1; j++) {
    for (int i = 0; i < DIM; i++) {
      _A.block(24 + i + 3 * j, j * 24 + i * 8, 1, N_ORDER + 1) = pos_1d;
      _ub(24 + i + 3 * j) = _waypoints[j](i);
    }
  }
}

void MiniSnap::getContinuityConstraint() {
  int M = N_ORDER + 1;
  int K = DIM * M;           // 3*8
  int S = 24 + 3 * (N - 1);  // start position
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;

  for (int j = 0; j < N - 1; j++) {
    for (int i = 0; i < DIM; i++) {
      _A.block(S + 0 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = pos_1d;
      _A.block(S + 1 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = vel_1d;
      _A.block(S + 2 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = acc_1d;
      _A.block(S + 3 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = jer_1d;
      _A(S + 0 + 4 * i + 12 * j, 0 + K * (j + 1) + i * M) = -1;
      _A(S + 1 + 4 * i + 12 * j, 1 + K * (j + 1) + i * M) = -1;
      _A(S + 2 + 4 * i + 12 * j, 2 + K * (j + 1) + i * M) = -2;
      _A(S + 3 + 4 * i + 12 * j, 3 + K * (j + 1) + i * M) = -6;
    }
  }
  // std::cout << _ub << std::endl;
  // std::cout << _A.block(0, 0, 24, 360) << std::endl;
  // std::cout << _A.block(15 * N - 15, 3*8*N - 48, 24, 48) << std::endl;
}

bool MiniSnap::solveQP() {
  IOSQP solver;
  ROS_INFO("[TrajOpt] start solving");
  Eigen::VectorXd q;
  q.resize(N * (N_ORDER + 1) * DIM);
  q.setZero();
  SparMat Q = _Q.sparseView();
  SparMat A = _A.sparseView();
  solver.setMats(Q, q, A, _lb, _ub, 1e-3, 1e-3);
  solver.solve();
  _x = solver.getPrimalSol();
  return true;
}

/**
 * @brief generate Cost and Constraints and optimize problem
 *
 * @return true
 * @return false
 */
bool MiniSnap::optimize() {
  getCostFunc();
  ROS_INFO("[TrajOpt] Generated Cost Func");
  getHeadTailConstraint();
  getWaypointsConstraint();
  ROS_INFO("[TrajOpt] Generated WaypointsConstraint");
  getContinuityConstraint();
  ROS_INFO("[TrajOpt] Generated ContinuityConstraint");
  _lb = _ub;
  bool isSuccess = solveQP();
  return isSuccess;
}

void MiniSnap::getTrajectory(Trajectory *traj) {
  traj->setDuration(_timeAlloc);
  traj->setCoefficient(_x);
}

double MiniSnap::getMinimumCost() const {
  return _x.transpose() * _Q * _x;
}


/***************************************/
/********** Corridor MiniSnap **********/
/***************************************/

/** Jing Chen, Tianbo Liu and Shaojie Shen, "Online generation of collision-free
 * trajectories for quadrotor flight in unknown cluttered environments," 2016
 * IEEE International Conference on Robotics and Automation (ICRA), 2016, pp.
 * 1476-1483, doi: 10.1109/ICRA.2016.7487283.
 */

void CorridorMiniSnap::reset(
    const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
    const std::vector<double> &timeAlloc,
    const std::vector<Eigen::Matrix<double, 6, -1>> &corridors) {
  ROS_INFO_STREAM("[TrajOpt] head state:\n" << head);
  ROS_INFO_STREAM("[TrajOpt] tail state:\n" << tail);
  _headPVA = head;
  _tailPVA = tail;
  ROS_INFO_STREAM("[TrajOpt] timeAlloc:" << timeAlloc.size());
  _timeAlloc = timeAlloc;
  _Polygons = corridors;
  N = timeAlloc.size();
  int S = N * (N_ORDER + 1) * DIM;  // number of variables
  _x.resize(S);
  _Q.resize(S, S);
  _Q.setZero();

  n_hyperplanes = 0;
  for (int i = 0; i < corridors.size() - 1; i++) {
    int c_prev = corridors[i].cols();
    int c_next = corridors[i + 1].cols();
    n_hyperplanes += c_prev;
    n_hyperplanes += c_next;
  }

  /**
   * @brief constraint sampled trajectory points
   */
  // int M = DIM * 4 * 2 + 2 * N_POLYHEDRA * (N - 1) + DIM * 4 * (N - 1);
  int M = DIM * 4 * 2 + DIM * 4 * (N - 1) + n_hyperplanes;
  _A.resize(M, S);
  _A.setZero();
  _ub.resize(M);  // inherited b as upper bound
  _ub.setZero();
  _lb.resize(M);  // lower bound
  _lb.setZero();
}

/**
 * @brief
 * _Polygons[j]: 6-by-4 matrix: [direction, point] x 6
 */
void CorridorMiniSnap::getTransitionConstraint() {
  int SR = 24;
  int row_index = SR;
  int N_PIECE =
      DIM * (N_ORDER + 1);  // number of coeffs in single piece (8*3=24)
  int C = (N_ORDER + 1);    // number of coeffs in single dimension
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;  // end position
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;

  for (int j = 0; j < N - 1; j++) {
    Eigen::MatrixXd this_corridor = _Polygons[j];
    Eigen::MatrixXd next_corridor = _Polygons[j + 1];
    for (int i = 0; i < this_corridor.cols(); i++) {
      Eigen::VectorXd v = this_corridor.col(i);
      // int row_index = SR + i + 2 * N_POLYHEDRA * j;
      _A.block(row_index, j * N_PIECE + 0, 1, C) = pos_1d * v(0);
      _A.block(row_index, j * N_PIECE + C, 1, C) = pos_1d * v(1);
      _A.block(row_index, j * N_PIECE + C * 2, 1, C) = pos_1d * v(2);
      _ub(row_index) = v(0) * v(3) + v(1) * v(4) + v(2) * v(5);
      _lb(row_index) = -OSQP_INFTY;
      row_index++;
    }
    for (int i = 0; i < next_corridor.cols(); i++) {
      Eigen::VectorXd v = next_corridor.col(i);
      // int row_index = SR + i + 2 * N_POLYHEDRA * j + N_POLYHEDRA;
      _A.block(row_index, j * N_PIECE + 0, 1, C) = pos_1d * v(0);
      _A.block(row_index, j * N_PIECE + C, 1, C) = pos_1d * v(1);
      _A.block(row_index, j * N_PIECE + C * 2, 1, C) = pos_1d * v(2);
      _ub(row_index) = v(0) * v(3) + v(1) * v(4) + v(2) * v(5);
      _lb(row_index) = -OSQP_INFTY;
      row_index++;
    }
  }
}

void CorridorMiniSnap::getContinuityConstraint() {
  int M = N_ORDER + 1;
  int K = DIM * M;              // 3*8
  int SR = 24 + n_hyperplanes;  // start row
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;

  for (int j = 0; j < N - 1; j++) {
    for (int i = 0; i < DIM; i++) {
      _A.block(SR + 0 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = pos_1d;
      _A.block(SR + 1 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = vel_1d;
      _A.block(SR + 2 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = acc_1d;
      _A.block(SR + 3 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = jer_1d;
      _A(SR + 0 + 4 * i + 12 * j, 0 + K * (j + 1) + i * M) = -1;
      _A(SR + 1 + 4 * i + 12 * j, 1 + K * (j + 1) + i * M) = -1;
      _A(SR + 2 + 4 * i + 12 * j, 2 + K * (j + 1) + i * M) = -2;
      _A(SR + 3 + 4 * i + 12 * j, 3 + K * (j + 1) + i * M) = -6;
    }
  }
  // std::cout << _ub << std::endl;
  // std::cout << _A.block(0, 0, 24, 360) << std::endl;
  // std::cout << _A.block(15 * N - 15, 3*8*N - 48, 24, 48) << std::endl;
}

void CorridorMiniSnap::getHeadTailConstraint() {
  /* constraints for starting states*/
  for (int i = 0; i < DIM; i++) {
    _A(0 + 4 * i, 0 + 8 * i) = 1;
    _A(1 + 4 * i, 1 + 8 * i) = 1;
    _A(2 + 4 * i, 2 + 8 * i) = 2;
    _A(3 + 4 * i, 3 + 8 * i) = 6;
    _ub(0 + 4 * i) = _headPVA(i, 0);
    _ub(1 + 4 * i) = _headPVA(i, 1);
    _ub(2 + 4 * i) = _headPVA(i, 2);
    _ub(3 + 4 * i) = 0;
    _lb(0 + 4 * i) = _headPVA(i, 0);
    _lb(1 + 4 * i) = _headPVA(i, 1);
    _lb(2 + 4 * i) = _headPVA(i, 2);
    _lb(3 + 4 * i) = 0;
  }

  /* constraints for end states */
  int M = (N - 1) * (N_ORDER + 1) * DIM;
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;
  for (int i = 0; i < DIM; i++) {
    // std::cout << _A.rows() << ' ' << _A.cols() << ' ' << M + 8 * i <<
    // std::endl;
    _A.block(12 + 0 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = pos_1d;
    _A.block(12 + 1 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = vel_1d;
    _A.block(12 + 2 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = acc_1d;
    _A.block(12 + 3 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = jer_1d;
    _ub(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _ub(12 + 1 + 4 * i) = _tailPVA(i, 1);
    _ub(12 + 2 + 4 * i) = _tailPVA(i, 2);
    _ub(12 + 3 + 4 * i) = 0;
    _lb(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _lb(12 + 1 + 4 * i) = _tailPVA(i, 1);
    _lb(12 + 2 + 4 * i) = _tailPVA(i, 2);
    _lb(12 + 3 + 4 * i) = 0;
  }
}

bool CorridorMiniSnap::primarySolveQP() {
  IOSQP solver;
  ROS_INFO("[TrajOpt] start solving");
  Eigen::VectorXd q;
  q.resize(N * (N_ORDER + 1) * DIM);
  q.setZero();
  SparMat Q = _Q.sparseView();
  SparMat A = _A.sparseView();
  solver.setMats(Q, q, A, _lb, _ub, 1e-3, 1e-3);
  solver.solve();
  _x = solver.getPrimalSol();
  return true;
}

bool CorridorMiniSnap::optimize() {
  getCostFunc();
  ROS_INFO("[TrajOpt] Generated Cost Func");
  getHeadTailConstraint();
  ROS_INFO("[TrajOpt] Generated Head Tail Constraint");
  getTransitionConstraint();
  ROS_INFO("[TrajOpt] Generated Transitional Constraint");
  getContinuityConstraint();
  ROS_INFO("[TrajOpt] Generated Continuity Constraint");
  // getCorridorConstraint();
  // std::cout << "Generated Corridor Constraint" << std::endl;
  bool isSuccess = primarySolveQP();
  return isSuccess;
}

bool CorridorMiniSnap::reOptimize() {
  ROS_INFO("[TrajOpt] Solve new QP problem");
  bool isSuccess = primarySolveQP();
  return isSuccess;
}

void CorridorMiniSnap::getCostFunc() {
  /* for single piece, single dimension */
  int D = N_ORDER + 1;  // size of matrix Q
  Eigen::Matrix<double, N_ORDER + 1, N_ORDER + 1> Q;
  for (int i = 0; i <= N_ORDER; i++) {
    for (int j = 0; j <= N_ORDER; j++) {
      if (i < 4 || j < 4) {
        Q(i, j) = 0;
      }
      if (i + j > N_ORDER) {
        Q(i, j) = i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) *
                  (j - 3) / (i + j - N_ORDER);
      }
    }
  }
  /* iterate all dimensions and all pieces */
  for (int i = 0; i < N * DIM; i++) {
    _Q.block(i * D, i * D, D, D) = Q;
  }
}

typedef Eigen::Matrix<double, DIM, N_ORDER + 1> CoeffMatrix;

/**
 * @brief derivative calculate
 *
 * @param coeff
 * @return Eigen::Matrix<double, DIM, N_ORDER + 1>
 */
Eigen::Matrix<double, DIM, N_ORDER> derivative(CoeffMatrix coeff) {
  Eigen::Matrix<double, DIM, N_ORDER> der;
  for (int i = 0; i < DIM; i++) {
    for (int j = 0; j < N_ORDER; j++) {
      der(i, j) = coeff(i, j + 1) * (j + 1);
    }
  }
  return der;
}

/**
 * @brief tangent curve based constraint refinement
 *
 * @param traj
 * @return true
 * @return false
 * @author Moji Shi
 */
bool CorridorMiniSnap::isCorridorSatisfied(Trajectory &traj) {
  bool isSatisfied = true;
  std::vector<Eigen::VectorXd> constraints;
  int C = N_ORDER + 1;
  int N_PIECES = DIM * C;
  int S = N * (N_ORDER + 1) * DIM;
  double delta = 0.1;  // boundary margin

  for (int idx = 0; idx < N; idx++) {
    Eigen::MatrixXd polyhedra = _Polygons[idx];
    CoeffMatrix coeff = traj[idx].getCoefficient();
    Eigen::Matrix<double, DIM, N_ORDER> coeff_dot = derivative(coeff);

    for (int i = 0; i < polyhedra.cols(); i++) {
      Eigen::Vector3d n_vec = polyhedra.block<3, 1>(0, i);
      Eigen::Vector3d p = polyhedra.block<3, 1>(3, i);
      Eigen::VectorXd coeff_solver_reverse(7), coeff_solver(7);
      coeff_solver_reverse = n_vec.transpose() * coeff_dot;
      for (int j = 0; j < 7; j++) {
        coeff_solver(j) = coeff_solver_reverse(6 - j);
      }

      std::set<double> allRoots =
          RootFinder::solvePolynomial(coeff_solver, 0, 1, 0.0000001);

      for (auto itr = allRoots.begin(); itr != allRoots.end(); itr++) {
        double t = *itr;

        Eigen::Vector3d pos = traj[idx].getPos(t);

        if (n_vec.dot(p - pos) < 0) {
          isSatisfied = false;
          /// debug ///
          // std::cout << "tangent point:" << std::endl;
          // std::cout << "piece\t" << idx << std::endl;
          // std::cout << "time\t" <<t<< "\tpos " << pos.transpose() <<
          // std::endl;

          /* add a single corridor constraint */
          Eigen::Matrix<double, 1, N_ORDER + 1> d;
          d << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6),
              pow(t, 7);
          Eigen::Matrix<double, 1, 3 * (N_ORDER + 1)> d3;
          d3 << n_vec(0) * d, n_vec(1) * d, n_vec(2) * d;
          p = p - delta * n_vec;  // constraint with boundary margin
          Eigen::VectorXd Ab(S + 1);
          Ab.setZero();
          Ab.segment(idx * N_PIECES, N_PIECES) = d3;
          Ab(S) = n_vec(0) * p(0) + n_vec(1) * p(1) + n_vec(2) * p(2);
          constraints.push_back(Ab);
        }
      }
    }
  }

  /* load constraints to A matrix */
  int n = constraints.size();
  int ROW = _A.rows();
  _A.conservativeResize(ROW + n, Eigen::NoChange);
  _ub.conservativeResize(ROW + n);
  _lb.conservativeResize(ROW + n);
  for (int i = 0; i < n; i++) {
    _A.row(ROW + i) = constraints[i].head(S);
    _ub(ROW + i) = constraints[i](S);
    _lb(ROW + i) = -OSQP_INFTY;
  }

  std::cout << "\033[42m"
            << "Get new constriants:\tttl size: " << _ub.rows() << "\033[0m"
            << std::endl;

  return isSatisfied;
}

void CorridorMiniSnap::getCorridorConstraint() {
  std::cout << "TODO" << std::endl;
}

void CorridorMiniSnap::getTrajectory(Trajectory *traj) {
  traj->setDuration(_timeAlloc);
  traj->setCoefficient(_x);
  std::cout << (*traj)[3].getCoefficient().row(0) << std::endl;
}

double CorridorMiniSnap::getMinimumCost() const {
  return _x.transpose() * _Q * _x;
}


/***************************************/
/***** Corridor MiniSnap Original ******/
/***************************************/

/** D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control
 * for quadrotors," 2011 IEEE International Conference on Robotics and
 * Automation, 2011, pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.
 */

void CorridorMiniSnapOriginal::reset(
    const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
    const std::vector<double> &timeAlloc,
    const std::vector<Eigen::Matrix<double, 6, -1>> &corridors) {
  ROS_INFO_STREAM("[TrajOpt] head state:\n" << head);
  ROS_INFO_STREAM("[TrajOpt] tail state:\n" << tail);
  _headPVA = head;
  _tailPVA = tail;
  ROS_INFO_STREAM("[TrajOpt]timeAlloc:" << timeAlloc.size());
  _timeAlloc = timeAlloc;
  _Polygons = corridors;
  N = timeAlloc.size();
  int S = N * (N_ORDER + 1) * DIM;  // number of variables
  _x.resize(S);
  _Q.resize(S, S);
  _Q.setZero();

  n_hyperplanes = 0;
  n_constraints = corridors[0].cols();
  for (int i = 0; i < corridors.size() - 1; i++) {
    int c_prev = corridors[i].cols();
    int c_next = corridors[i + 1].cols();
    n_hyperplanes += c_prev;
    n_hyperplanes += c_next;
    n_constraints += c_next;
  }

  /**
   * @brief constraint sampled trajectory points
   */
  // std::cout << "M:  " << DIM * 4 * 2 + DIM * 4 * (N - 1) + n_hyperplanes << std::endl;
  // std::cout << "M:  " << n_constraints * N_SAMPLES << std::endl;
  int M = DIM * 4 * 2 + DIM * 4 * (N - 1) + n_hyperplanes +
          n_constraints * N_SAMPLES;
  _A.resize(M, S);
  _A.setZero();
  _ub.resize(M);  // inherited b as upper bound
  _ub.setZero();
  _lb.resize(M);  // lower bound
  _lb.setZero();
}

/**
 * @brief
 * _Polygons[j]: 6-by-4 matrix: [direction, point] x 6
 */
void CorridorMiniSnapOriginal::getTransitionConstraint() {
  int SR = 24;
  int row_index = SR;
  int N_PIECE =
      DIM * (N_ORDER + 1);  // number of coeffs in single piece (8*3=24)
  int C = (N_ORDER + 1);    // number of coeffs in single dimension
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;  // end position
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;

  for (int j = 0; j < N - 1; j++) {
    Eigen::MatrixXd this_corridor = _Polygons[j];
    Eigen::MatrixXd next_corridor = _Polygons[j + 1];
    for (int i = 0; i < this_corridor.cols(); i++) {
      Eigen::VectorXd v = this_corridor.col(i);
      // int row_index = SR + i + 2 * N_POLYHEDRA * j;
      _A.block(row_index, j * N_PIECE + 0, 1, C) = pos_1d * v(0);
      _A.block(row_index, j * N_PIECE + C, 1, C) = pos_1d * v(1);
      _A.block(row_index, j * N_PIECE + C * 2, 1, C) = pos_1d * v(2);
      _ub(row_index) = v(0) * v(3) + v(1) * v(4) + v(2) * v(5);
      _lb(row_index) = -OSQP_INFTY;
      row_index++;
    }
    for (int i = 0; i < next_corridor.cols(); i++) {
      Eigen::VectorXd v = next_corridor.col(i);
      // int row_index = SR + i + 2 * N_POLYHEDRA * j + N_POLYHEDRA;
      _A.block(row_index, j * N_PIECE + 0, 1, C) = pos_1d * v(0);
      _A.block(row_index, j * N_PIECE + C, 1, C) = pos_1d * v(1);
      _A.block(row_index, j * N_PIECE + C * 2, 1, C) = pos_1d * v(2);
      _ub(row_index) = v(0) * v(3) + v(1) * v(4) + v(2) * v(5);
      _lb(row_index) = -OSQP_INFTY;
      row_index++;
    }
  }
  // std::cout << row_index << std::endl;
}

void CorridorMiniSnapOriginal::getContinuityConstraint() {
  int M = N_ORDER + 1;
  int K = DIM * M;              // 3*8
  int SR = 24 + n_hyperplanes;  // start row
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;

  for (int j = 0; j < N - 1; j++) {
    for (int i = 0; i < DIM; i++) {
      _A.block(SR + 0 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = pos_1d;
      _A.block(SR + 1 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = vel_1d;
      _A.block(SR + 2 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = acc_1d;
      _A.block(SR + 3 + 4 * i + 12 * j, i * M + j * K, 1, N_ORDER + 1) = jer_1d;
      _A(SR + 0 + 4 * i + 12 * j, 0 + K * (j + 1) + i * M) = -1;
      _A(SR + 1 + 4 * i + 12 * j, 1 + K * (j + 1) + i * M) = -1;
      _A(SR + 2 + 4 * i + 12 * j, 2 + K * (j + 1) + i * M) = -2;
      _A(SR + 3 + 4 * i + 12 * j, 3 + K * (j + 1) + i * M) = -6;
    }
  }
  // std::cout << _ub << std::endl;
  // std::cout << _A.block(0, 0, 24, 360) << std::endl;
  // std::cout << _A.block(15 * N - 15, 3*8*N - 48, 24, 48) << std::endl;
}

void CorridorMiniSnapOriginal::getHeadTailConstraint() {
  /* constraints for starting states*/
  for (int i = 0; i < DIM; i++) {
    _A(0 + 4 * i, 0 + 8 * i) = 1;
    _A(1 + 4 * i, 1 + 8 * i) = 1;
    _A(2 + 4 * i, 2 + 8 * i) = 2;
    _A(3 + 4 * i, 3 + 8 * i) = 6;
    _ub(0 + 4 * i) = _headPVA(i, 0);
    _ub(1 + 4 * i) = _headPVA(i, 1);
    _ub(2 + 4 * i) = _headPVA(i, 2);
    _ub(3 + 4 * i) = 0;
    _lb(0 + 4 * i) = _headPVA(i, 0);
    _lb(1 + 4 * i) = _headPVA(i, 1);
    _lb(2 + 4 * i) = _headPVA(i, 2);
    _lb(3 + 4 * i) = 0;
  }

  /* constraints for end states */
  int M = (N - 1) * (N_ORDER + 1) * DIM;
  Eigen::Matrix<double, 1, N_ORDER + 1> pos_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> vel_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> acc_1d;
  Eigen::Matrix<double, 1, N_ORDER + 1> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;
  for (int i = 0; i < DIM; i++) {
    // std::cout << _A.rows() << ' ' << _A.cols() << ' ' << M + 8 * i <<
    // std::endl;
    _A.block(12 + 0 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = pos_1d;
    _A.block(12 + 1 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = vel_1d;
    _A.block(12 + 2 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = acc_1d;
    _A.block(12 + 3 + 4 * i, M + 8 * i, 1, N_ORDER + 1) = jer_1d;
    _ub(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _ub(12 + 1 + 4 * i) = _tailPVA(i, 1);
    _ub(12 + 2 + 4 * i) = _tailPVA(i, 2);
    _ub(12 + 3 + 4 * i) = 0;
    _lb(12 + 0 + 4 * i) = _tailPVA(i, 0);
    _lb(12 + 1 + 4 * i) = _tailPVA(i, 1);
    _lb(12 + 2 + 4 * i) = _tailPVA(i, 2);
    _lb(12 + 3 + 4 * i) = 0;
  }
}

bool CorridorMiniSnapOriginal::primarySolveQP() {
  IOSQP solver;
  ROS_INFO("[TrajOpt] start solving");
  Eigen::VectorXd q;
  q.resize(N * (N_ORDER + 1) * DIM);
  q.setZero();
  SparMat Q = _Q.sparseView();
  SparMat A = _A.sparseView();
  solver.setMats(Q, q, A, _lb, _ub, 1e-3, 1e-3);
  solver.solve();
  _x = solver.getPrimalSol();
  return true;
}

bool CorridorMiniSnapOriginal::optimize() {
  getCostFunc();
  ROS_INFO("[TrajOpt] Generated Cost Func");
  getHeadTailConstraint();
  ROS_INFO("[TrajOpt] Generated Head Tail Constraint");
  getTransitionConstraint();
  ROS_INFO("[TrajOpt] Generated Transitional Constraint");
  getContinuityConstraint();
  ROS_INFO("[TrajOpt] Generated Continuity Constraint");
  getCorridorConstraint();
  ROS_INFO("[TrajOpt] Generated Corridor Constraint");
  bool isSuccess = primarySolveQP();
  return isSuccess;
}

void CorridorMiniSnapOriginal::getCostFunc() {
  /* for single piece, single dimension */
  int D = N_ORDER + 1;  // size of matrix Q
  Eigen::Matrix<double, N_ORDER + 1, N_ORDER + 1> Q;
  for (int i = 0; i <= N_ORDER; i++) {
    for (int j = 0; j <= N_ORDER; j++) {
      if (i < 4 || j < 4) {
        Q(i, j) = 0;
      }
      if (i + j > N_ORDER) {
        Q(i, j) = i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) *
                  (j - 3) / (i + j - N_ORDER);
      }
    }
  }
  /* iterate all dimensions and all pieces */
  for (int i = 0; i < N * DIM; i++) {
    _Q.block(i * D, i * D, D, D) = Q;
  }
}

/**
 * @brief sample based constraint refinement
 *
 * @param traj
 * @return true
 * @return false
 * @author Siyuan Wu
 */
bool CorridorMiniSnapOriginal::isCorridorSatisfied(Trajectory &traj) {
  bool isSatisfied = true;
  int C = N_ORDER + 1;
  int N_PIECES = DIM * C;
  int S = N * (N_ORDER + 1) * DIM;
  // std::cout << "_A" << _A.rows() << std::endl;
  std::vector<Eigen::VectorXd> constraints;

  /* iterate over all pieces */
  for (int idx = 0; idx < N; idx++) {
    Eigen::MatrixXd polyhedra = _Polygons[idx];

    /* iterate over uniformly sample positions */
    for (double dt = 0; dt < 1; dt += 0.05) {
      Eigen::Vector3d pos = traj[idx].getPos(dt);

      /* iterate over all polygons */
      for (int i = 0; i < polyhedra.cols(); i++) {
        Eigen::Vector3d a = polyhedra.block<3, 1>(0, i);
        Eigen::Vector3d p = polyhedra.block<3, 1>(3, i);
        if (a.dot(p - pos) < 0) {
          isSatisfied = false;

          // std::cout << "piece\t" << idx << std::endl;
          // std::cout << "time\t" << dt << "\tpos " << pos.transpose()
          //           << std::endl;

          /* add a single corridor constraint */
          Eigen::Matrix<double, 1, N_ORDER + 1> d;
          d << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5),
              pow(dt, 6), pow(dt, 7);
          Eigen::Matrix<double, 1, 3 * (N_ORDER + 1)> d3;
          d3 << a(0) * d, a(1) * d, a(2) * d;

          Eigen::VectorXd Ab(S + 1);
          Ab.setZero();
          Ab.segment(idx * N_PIECES, N_PIECES) = d3;
          Ab(S) = a(0) * p(0) + a(1) * p(1) + a(2) * p(2);
          constraints.push_back(Ab);
        }
      }
    }
  }
  /* load constraints to A matrix */
  int n = constraints.size();
  // std::cout << "_A " << n << std::endl;
  int ROW = _A.rows();
  _A.conservativeResize(ROW + n, Eigen::NoChange);
  _ub.conservativeResize(ROW + n);
  _lb.conservativeResize(ROW + n);
  for (int i = 0; i < n; i++) {
    _A.row(ROW + i) = constraints[i].head(S);
    _ub(ROW + i) = constraints[i](S);
    _lb(ROW + i) = -OSQP_INFTY;
  }

  // std::cout << "\033[42m"
  //           << "Get new constriants:\tttl size: " << _ub.rows() << "\033[0m"
  //           << std::endl;
  // std::cout << _A.block(_A.rows() - n, 0, n, _A.cols()) << std::endl;
  // std::cout << _ub.tail(n) << std::endl;
  // std::cout << "_A" << _A.rows() << std::endl;
  return true;
}

void CorridorMiniSnapOriginal::getCorridorConstraint() {
  int row_index = DIM * 4 * 2 + DIM * 4 * (N - 1) + n_hyperplanes;
  int C = N_ORDER + 1;
  int S = N * (N_ORDER + 1) * DIM;
  int N_PIECES = DIM * C;
  double delta = 1.0 / N_SAMPLES;
  /* iterate over all pieces */
  for (int idx = 0; idx < N; idx++) {
    Eigen::MatrixXd polyhedra = _Polygons[idx];

    /* iterate over uniformly sample positions */
    for (double dt = 0; dt < 1; dt += delta) {
      /* iterate over all polygons */
      for (int i = 0; i < polyhedra.cols(); i++) {
        Eigen::Vector3d a = polyhedra.block<3, 1>(0, i);
        Eigen::Vector3d p = polyhedra.block<3, 1>(3, i);

        // std::cout << "piece\t" << idx << std::endl;
        // std::cout << "time\t" << dt << std::endl;

        /* add a single corridor constraint */
        Eigen::Matrix<double, 1, N_ORDER + 1> d;
        d << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5), pow(dt, 6),
            pow(dt, 7);
        Eigen::Matrix<double, 1, 3 * (N_ORDER + 1)> d3;
        d3 << a(0) * d, a(1) * d, a(2) * d;

        Eigen::VectorXd A(S);
        A.setZero();
        A.segment(idx * N_PIECES, N_PIECES) = d3;
        _A.row(row_index) = A;
        _ub(row_index) = a(0) * p(0) + a(1) * p(1) + a(2) * p(2);
        _lb(row_index) = -OSQP_INFTY;
        row_index++;
      }
    }
    // std::cout << row_index << std::endl;
  }
}

void CorridorMiniSnapOriginal::getTrajectory(Trajectory *traj) {
  traj->setDuration(_timeAlloc);
  traj->setCoefficient(_x);
  // std::cout << (*traj)[3].getCoefficient().row(0) << std::endl;
}

double CorridorMiniSnapOriginal::getMinimumCost() const {
  return _x.transpose() * _Q * _x;
}