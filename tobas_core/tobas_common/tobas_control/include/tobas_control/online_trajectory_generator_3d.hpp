#pragma once

#include <eigen3/Eigen/Core>

#include "./online_trajectory_generator.hpp"

namespace ctrl
{
class OnlineTrajectoryGenerator3d
{
public:
  explicit OnlineTrajectoryGenerator3d();

  Eigen::Vector3d getCommandPosition() const;
  Eigen::Vector3d getCommandVelocity() const;
  Eigen::Vector3d getCommandAcceleration() const;

  Eigen::Vector3d getTargetPosition() const;
  Eigen::Vector3d getTargetVelocity() const;
  Eigen::Vector3d getTargetAcceleration() const;

  void setTargetPosition(const Eigen::Vector3d& tar_pos);
  void setTargetVelocity(const Eigen::Vector3d& tar_vel);
  void setTargetAcceleration(const Eigen::Vector3d& tar_acc);

  void setMinVelocity(size_t idx, double min_vel);
  void setMaxVelocity(size_t idx, double max_vel);
  void setMinAcceleration(size_t idx, double min_acc);
  void setMaxAcceleration(size_t idx, double max_acc);
  void setMaxJerk(size_t idx, double max_jerk);

  void setSpeedOverride(size_t idx, double speed_override);

  void update(double dt, const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc);

  void update(double dt);

private:
  std::array<OnlineTrajectoryGenerator, 3> otg_;
};
}  // namespace ctrl
