#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_trajectory_commander/position_yaw.hpp"

namespace tobas_trajectory_commander
{
class FollowPositionYawTrajectoryServerNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<FollowPositionYawTrajectoryServer> node_;
};
}  // namespace tobas_trajectory_commander
