#pragma once

#include <cmath>

namespace ctrl
{
/**
 * @brief 制約を満たしつつ最短時間で目標状態に到達する軌道をオンラインで更新する．
 * @note バンバン制御は安定余裕が無いので，遅延，モデル化誤差を含む実環境でフィードバックループとして使うのは難しい．
 */
class OnlineTrajectoryGenerator
{
public:
  explicit OnlineTrajectoryGenerator();

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

  void setSpeedOverride(double speed_override);

  /* 状態フィードバックを含む更新． */
  void update(double dt, double cur_pos, double cur_vel, double cur_acc);

  /* 状態フィードバックを含まない更新． */
  void update(double dt);

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
  double min_vel_ = NAN;
  double max_vel_ = NAN;
  double min_acc_ = NAN;
  double max_acc_ = NAN;
  double max_jerk_ = NAN;

  double speed_override_ = 1.;
};

inline double OnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return traj_pos_;
}

inline double OnlineTrajectoryGenerator::getTrajectoryVelocity() const
{
  return traj_vel_;
}

inline double OnlineTrajectoryGenerator::getTrajectoryAcceleration() const
{
  return traj_acc_;
}

inline void OnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  tar_pos_ = tar_pos;
}

inline void OnlineTrajectoryGenerator::setTargetVelocity(double tar_vel)
{
  tar_vel_ = tar_vel;
}

inline void OnlineTrajectoryGenerator::setTargetAcceleration(double tar_acc)
{
  tar_acc_ = tar_acc;
}

}  // namespace ctrl
