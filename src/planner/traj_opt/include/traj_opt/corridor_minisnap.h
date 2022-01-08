/**
 * @file corridor_minisnap.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-22
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef CORRIDOR_MINI_SNAP_H_
#define CORRIDOR_MINI_SNAP_H_
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <iostream>
#include <traj_opt/iosqp.hpp>
#include <vector>

#define N_ORDER 7      // order of polynomial trajectory
#define D_ORDER 4      // order of maximum derivative (4 for minisnap)
#define DIM 3          // number of dimensions in Cspace
#define N_POLYHEDRA 6  // number of polygons in polyhedra
#define N_SAMPLES 5   // number of samples for each pieces (Mellinger et al.)

namespace traj_opt {

/** @brief sparse matrix  */
typedef Eigen::SparseMatrix<double> SparMat;

/** @brief triplet, for adding element to sparse matrix efficiently */
typedef Eigen::Triplet<double> Triplet;

/**
 * @brief A piece of polynomial trajectory
 * use relative time t_ = t / T
 */
class PolyPiece {
 private:
  double _duration;
  Eigen::Matrix<double, DIM, N_ORDER + 1> _coeffs;

 public:
  PolyPiece() {}
  ~PolyPiece() {}
  void setup(const double t);
  void setup(const Eigen::MatrixXd coeffs);
  void setup(const double t, const Eigen::MatrixXd coeffs);
  Eigen::Vector3d getPos(double t) const;
  Eigen::Vector3d getVel(double t) const;
  Eigen::Vector3d getAcc(double t) const;
  double getDuration() const;
  Eigen::Matrix<double, DIM, N_ORDER + 1> getCoefficient() const;
};

class Trajectory {
 private:
  typedef std::vector<PolyPiece> Pieces;
  int N;
  Pieces _pieces;

 public:
  Trajectory() {}
  ~Trajectory() {}
  void setDuration(const std::vector<double> &t);
  void setCoefficient(const Eigen::VectorXd &x);
  inline void locatePiece(const double &t0, double &t, int &idx) const;
  Eigen::Matrix3Xd getPositions() const;
  Eigen::Vector3d getPos(double t) const;
  Eigen::Vector3d getVel(double t) const;
  Eigen::Vector3d getAcc(double t) const;
  double getMaxVelRate() const;
  double getDuration() const;
  int getPieceNum() const;
  const PolyPiece &operator[](int i) const { return _pieces[i]; }
  PolyPiece &operator[](int i) { return _pieces[i]; }
};

// typedef Trajectory Traj;

class MiniSnap {
 protected:
  int N;                     // number of pieces
  Eigen::Matrix3d _headPVA;  // head's pos, vel, acc
  Eigen::Matrix3d _tailPVA;  // tail's pos, vel, acc
  Eigen::MatrixXd _Q;
  Eigen::MatrixXd _A;
  Eigen::VectorXd _x;  // solutions
  Eigen::VectorXd _ub;
  Eigen::VectorXd _lb;
  std::vector<Eigen::Vector3d> _waypoints;
  std::vector<double> _timeAlloc;

 public:
  MiniSnap() {}
  ~MiniSnap() {}
  void reset(const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
             const std::vector<Eigen::Vector3d> &waypoints,
             const std::vector<double> &timeAlloc);
  bool optimize();
  bool solveQP();
  void getCostFunc();
  void getHeadTailConstraint();
  void getContinuityConstraint();
  void getWaypointsConstraint();
  void getTrajectory(Trajectory *traj);
};

class CorridorMiniSnapOriginal {
 private:
  int N;  // number of pieces
  int n_hyperplanes;
  int n_constraints;
  Eigen::Matrix3d _headPVA;  // head's pos, vel, acc
  Eigen::Matrix3d _tailPVA;  // tail's pos, vel, acc
  Eigen::MatrixXd _Q;
  Eigen::MatrixXd _A;
  Eigen::VectorXd _x;  // solutions
  Eigen::VectorXd _ub;
  Eigen::VectorXd _lb;
  std::vector<double> _timeAlloc;
  std::vector<Eigen::Matrix<double, 6, -1>> _Polygons;

 public:
  CorridorMiniSnapOriginal() {}
  ~CorridorMiniSnapOriginal() {}
  void reset(const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
             const std::vector<double> &timeAlloc,
             const std::vector<Eigen::Matrix<double, 6, -1>> &corridors);

  void getCostFunc();
  void getCorridorConstraint();
  void getTransitionConstraint();
  void getContinuityConstraint();
  void getHeadTailConstraint();

  bool optimize();
  bool primarySolveQP();
  bool reOptimize();
  // inline bool isCorridorSatisfied(const Eigen::Vector3d & pos, int idx,
  // double t);
  bool isCorridorSatisfied(Trajectory &traj);
  void getTrajectory(Trajectory *traj);
};

class CorridorMiniSnap {
 private:
  int N;  // number of pieces
  int n_hyperplanes;
  Eigen::Matrix3d _headPVA;  // head's pos, vel, acc
  Eigen::Matrix3d _tailPVA;  // tail's pos, vel, acc
  Eigen::MatrixXd _Q;
  Eigen::MatrixXd _A;
  Eigen::VectorXd _x;  // solutions
  Eigen::VectorXd _ub;
  Eigen::VectorXd _lb;
  std::vector<double> _timeAlloc;
  std::vector<Eigen::Matrix<double, 6, -1>> _Polygons;

 public:
  CorridorMiniSnap() {}
  ~CorridorMiniSnap() {}
  void reset(const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
             const std::vector<double> &timeAlloc,
             const std::vector<Eigen::Matrix<double, 6, -1>> &corridors);

  void getCostFunc();
  void getCorridorConstraint();
  void getTransitionConstraint();
  void getContinuityConstraint();
  void getHeadTailConstraint();

  bool optimize();
  bool primarySolveQP();
  bool reOptimize();
  // inline bool isCorridorSatisfied(const Eigen::Vector3d & pos, int idx,
  // double t);
  bool isCorridorSatisfied(Trajectory &traj);
  void getTrajectory(Trajectory *traj);
};

};  // namespace traj_opt

#endif  // CORRIDOR_MINI_SNAP_H_
