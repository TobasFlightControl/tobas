#pragma once

#include <cmath>

namespace traj
{
/**
 * @brief 制約を満たしつつ最短時間で目標状態に到達する軌道をオンラインで更新する．
 * @note バンバン制御は安定余裕が無いので，遅延，モデル化誤差を含む実環境でフィードバックループとして使うのは難しい．
 */
class JerkLimitedOnlineTrajectoryGenerator
{
public:
  explicit JerkLimitedOnlineTrajectoryGenerator();

  inline double getTrajectoryPosition() const;
  inline double getTrajectoryVelocity() const;
  inline double getTrajectoryAcceleration() const;

  inline void setTargetPosition(double tar_pos);
  inline void setTargetVelocity(double tar_vel);
  inline void setTargetAcceleration(double tar_acc);

  void setMinVelocity(double min_vel);
  void setMaxVelocity(double max_vel);
  void setMinAcceleration(double min_acc);
  void setMaxAcceleration(double max_acc);
  void setMaxJerk(double max_jerk);

  void update(double dt);

  void setTargetPointAndUpdate(double tar_pos, double tar_vel, double tar_acc, double dt);

  void resetCurrentTrajectoryPoint(double pos, double vel, double acc);

private:
  // Trajectory Point
  double traj_pos_ = 0.;
  double traj_vel_ = 0.;
  double traj_acc_ = 0.;

  // Target
  double tar_pos_ = 0.;
  double tar_vel_ = 0.;
  double tar_acc_ = 0.;

  // Limit
  double v_min_ = NAN;
  double v_max_ = NAN;
  double a_min_ = NAN;
  double a_max_ = NAN;
  double u_ = NAN;
};

inline double JerkLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return traj_pos_;
}

inline double JerkLimitedOnlineTrajectoryGenerator::getTrajectoryVelocity() const
{
  return traj_vel_;
}

inline double JerkLimitedOnlineTrajectoryGenerator::getTrajectoryAcceleration() const
{
  return traj_acc_;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  tar_pos_ = tar_pos;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetVelocity(double tar_vel)
{
  tar_vel_ = tar_vel;
}

inline void JerkLimitedOnlineTrajectoryGenerator::setTargetAcceleration(double tar_acc)
{
  tar_acc_ = tar_acc;
}
}  // namespace traj
