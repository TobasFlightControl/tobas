#pragma once

namespace ctrl
{
/* 制約を満たしつつ最短時間で目標状態に到達する起動をオンラインで更新する． */
class OnlineTrajectoryGenerator
{
public:
  explicit OnlineTrajectoryGenerator();

  double getCommandPosition() const;
  double getCommandVelocity() const;
  double getCommandAcceleration() const;

  void setTargetPosition(double tar_pos);
  void setTargetVelocity(double tar_vel);
  void setTargetAcceleration(double tar_acc);

  void setMinVelocity(double min_vel);
  void setMaxVelocity(double max_vel);
  void setMinAcceleration(double min_acc);
  void setMaxAcceleration(double max_acc);
  void setMaxJerk(double max_jerk);

  void setSpeedOverride(double speed_override);

  void update(double dt, double cur_pos, double cur_vel, double cur_acc);

private:
  // Command
  double cmd_pos_;
  double cmd_vel_;
  double cmd_acc_;

  // Target
  double tar_pos_ = 0.;
  double tar_vel_ = 0.;
  double tar_acc_ = 0.;

  // Limit
  double min_vel_ = 0.;
  double max_vel_ = 0.;
  double min_acc_ = 0.;
  double max_acc_ = 0.;
  double max_jerk_ = 0.;

  double speed_override_ = 1.;
};
}  // namespace ctrl
