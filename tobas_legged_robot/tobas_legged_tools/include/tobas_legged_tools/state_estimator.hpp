#pragma once

#include <tobas_kdl/quaternion.hpp>
#include <tobas_kdl/tree_fk_solver_vel.hpp>
#include <tobas_linear_control/kalman_filter.hpp>
#include <tobas_linear_control/c2d/rk4.hpp>

#include "./linear_dynamics.hpp"

namespace lr_tools
{
struct StateEstimatorConfig
{
  double variance_coef;
  int variance_exp;
};

/**
 * @brief 脚ロボットの状態推定器 (memo: 1-48)
 * @note (memo: 1-48)とは出力ベクトルの並びと制御入力が異なる． // TODO: 更新版をメモ
 */
class StateEstimator
{
  // 出力ベクトルのインデックス (脚数に依らない部分)
  static constexpr size_t kRollIdx = 0;
  static constexpr size_t kPitchIdx = kRollIdx + 1;
  static constexpr size_t kGyroIdx = kPitchIdx + 1;
  static constexpr size_t kGravIdx = kGyroIdx + 3;

  static constexpr double kFootGroundOffset = 0.02;  // [m] Footフレームの地面に対するオフセット
  static constexpr double kInitTrunkHeight = 0.3;    // [m] 体高の初期値

public:
  explicit StateEstimator(const kdl::Tree& tree, const std::vector<std::string>& foot_names);

  void updateInternalDataStructures();

  bool configure(const StateEstimatorConfig& cfg);

  void update(
    const kdl::Quaternion& W_Quat_B,
    const kdl::Vector& gyro_B,
    const kdl::JntArray& q,
    const kdl::JntArray& qd,
    const std::vector<bool>& is_stand,
    const std::vector<double>& contact_probs,
    const std::vector<kdl::Vector>& foot_forces,  // {footprint}から見た地面反力
    const std::vector<double>& foot_torques,      // {footprint}から見た地面反トルクのZ成分
    const double& dt);

  inline double getRoll() const;
  inline double getPitch() const;
  inline double getAlt() const;
  inline Eigen::Vector3d getGyro() const;
  inline Eigen::Vector3d getVel() const;

private:
  const std::vector<std::string> foot_names_;
  const size_t nc_;  // The number of contact points

  StateEstimatorConfig cfg_;

  kdl::TreeFkSolverVel fk_solver_;

  LinearDynamics cont_;
  ctrl::KalmanFilter kf_;
  ctrl::C2D_RK4 c2d_;

  double roll_, pitch_, yaw_;

  void initializeKalmanFilter();
  Eigen::MatrixXd makeCy();

  /* 出力ベクトルの高度に対応するインデックス． */
  inline size_t altIdx(size_t leg) const;

  /* 出力ベクトルの速度に対応するインデックス． */
  inline size_t velIdx(size_t leg) const;
};

inline double StateEstimator::getRoll() const
{
  return kf_.state()(LinearDynamics::kRollIdx);
}

inline double StateEstimator::getPitch() const
{
  return kf_.state()(LinearDynamics::kPitchIdx);
}

inline double StateEstimator::getAlt() const
{
  return kf_.state()(LinearDynamics::kAltIdx);
}

inline Eigen::Vector3d StateEstimator::getGyro() const
{
  return kf_.state().segment<3>(LinearDynamics::kGyroXIdx);
}

inline Eigen::Vector3d StateEstimator::getVel() const
{
  return kf_.state().segment<3>(LinearDynamics::kVelXIdx);
}

inline size_t StateEstimator::altIdx(size_t leg) const
{
  return kGravIdx + 1 + leg;
}

inline size_t StateEstimator::velIdx(size_t leg) const
{
  return kGravIdx + 1 + nc_ + 3 * leg;
}
}  // namespace lr_tools
