#pragma once

#include <cmath>

namespace ctrl
{
/**
 * @brief 制約を満たしつつ最短時間で目標状態に到達する軌道をオンラインで更新する．
 * @note バンバン制御は安定余裕が無いので，遅延，モデル化誤差を含む実環境でフィードバックループとして使うのは難しい．
 */
class VelocityLimitedOnlineTrajectoryGenerator
{
public:
  explicit VelocityLimitedOnlineTrajectoryGenerator();

  inline double getTrajectoryPosition() const;

  inline void setTargetPosition(double tar_pos);

  void setMaxVelocity(double max_vel);

  /* 状態フィードバックを含む更新． */
  void update(double dt, double cur_pos);

  /* 状態フィードバックを含まない更新． */
  void update(double dt);

  void resetCurrentTrajectoryPoint(double pos);

private:
  // Trajectory Point
  double traj_pos_ = 0.;

  // Target
  double tar_pos_ = 0.;

  // Limit
  double max_vel_ = NAN;
};

inline double VelocityLimitedOnlineTrajectoryGenerator::getTrajectoryPosition() const
{
  return traj_pos_;
}

inline void VelocityLimitedOnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  tar_pos_ = tar_pos;
}
}  // namespace ctrl
