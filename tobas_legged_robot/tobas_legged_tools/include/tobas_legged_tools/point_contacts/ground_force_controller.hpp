#pragma once

#include <tobas_std_tools/range.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>
#include <tobas_kdl/treeboundingboxsolver.hpp>
#include <tobas_linear_control/mpc/linear_dense.hpp>
#include <tobas_linear_control/c2d/rk4.hpp>

#include "./linear_dynamics.hpp"

namespace lr_tools
{
struct GroundForceControllerConfig
{
  double friction_coef;  // [-]

  double min_normal_force;  // [N]
  double max_normal_force;  // [N]

  double prediction_horizon;  // [s]
  size_t prediction_steps;

  double attitude_error_decay;  // [s]
  double height_error_decay;    // [s]
  double yawrate_error_decay;   // [s]
  double velocity_error_decay;  // [s]

  double attitude_weight;
  double height_weight;
  double gyro_weight;
  double velocity_weight;
  int force_weight_log10;
  int force_rate_weight_log10;
};

class GroundForceController
{
  static constexpr size_t kCtrlSize = 9;

public:
  explicit GroundForceController(const kdl::Tree& tree, const std::vector<std::string>& foot_names);

  void updateInternalDataStructures();

  bool configure(const GroundForceControllerConfig& cfg);

  void solve(
    const double& cur_z,
    const kdl::Vector& cur_vel,
    const kdl::Vector& cur_gyro,
    const double& tar_z,
    const double& tar_yawrate,
    const double& tar_vx,
    const double& tar_vy,
    const std::vector<double>& roll_pred,
    const std::vector<double>& pitch_pred,
    const std::vector<kdl::JntArray>& q_pred,
    const std::vector<std::vector<bool>>& is_stand_pred);

  inline const double& timeStep() const;

  inline Eigen::Vector3d optimalReactionForce(size_t leg) const;
  inline Eigen::Vector3d optimalDGyro() const;
  inline Eigen::Vector3d optimalAccel() const;

  inline double nextRoll() const;
  inline double nextPitch() const;
  inline double nextAltitude() const;
  inline Eigen::Vector3d nextGyro() const;
  inline Eigen::Vector3d nextVel() const;

private:
  const kdl::Tree& tree_;
  const std::vector<std::string> foot_names_;
  const size_t nc_;  // The number of contact points

  // Config
  double friction_coef_;
  tobas_std::Range<double> normal_force_range_;

  kdl::TreeJntToInertiaSolver inertia_solver_;
  kdl::TreeBoundingBoxSolver bb_solver_;
  PointContactsLinearDynamics cont_;
  ctrl::LinearDenseMPC mpc_;
  ctrl::C2D_RK4 c2d_;
  Eigen::VectorXd x_rate_;
  Eigen::VectorXd x_next_;

  void initializeMPC();
  double calcMass();
  double calcSizeScale();
  Eigen::MatrixXd makeCz();
  ctrl::LinearEquation makeInputConstraint();
};

inline const double& GroundForceController::timeStep() const
{
  return mpc_.time_step;
}

inline Eigen::Vector3d GroundForceController::optimalReactionForce(size_t leg) const
{
  return mpc_.optimalControlInput().segment<3>(3 * leg);
}

inline Eigen::Vector3d GroundForceController::optimalDGyro() const
{
  return x_rate_.segment<3>(PointContactsLinearDynamics::kGyroXIdx);
}

inline Eigen::Vector3d GroundForceController::optimalAccel() const
{
  return x_rate_.segment<3>(PointContactsLinearDynamics::kVelXIdx);
}

inline double GroundForceController::nextRoll() const
{
  return x_next_(PointContactsLinearDynamics::kRollIdx);
}

inline double GroundForceController::nextPitch() const
{
  return x_next_(PointContactsLinearDynamics::kPitchIdx);
}

inline double GroundForceController::nextAltitude() const
{
  return x_next_(PointContactsLinearDynamics::kAltIdx);
}

inline Eigen::Vector3d GroundForceController::nextGyro() const
{
  return x_next_.segment<3>(PointContactsLinearDynamics::kGyroXIdx);
}

inline Eigen::Vector3d GroundForceController::nextVel() const
{
  return x_next_.segment<3>(PointContactsLinearDynamics::kVelXIdx);
}
}  // namespace lr_tools
