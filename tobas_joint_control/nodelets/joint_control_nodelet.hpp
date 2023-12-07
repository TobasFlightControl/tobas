#pragma once

#include <nodelet/nodelet.h>

#include "../include/tobas_joint_control/joint_control_ros.hpp"

namespace tobas_joint_control
{
class JointControlNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::shared_ptr<JointControlRos> node_;
};
}  // namespace tobas_joint_control
