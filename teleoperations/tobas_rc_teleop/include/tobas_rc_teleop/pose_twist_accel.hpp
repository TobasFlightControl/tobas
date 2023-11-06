#pragma once

#include <Eigen/Core>

#include <dh_std_tools/range.hpp>
#include <dh_std_tools/first_order_filter.hpp>

#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RCInput.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PoseTwistAccelController : public BaseController
{
  using super = BaseController;

public:
  explicit PoseTwistAccelController();

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::PoseTwist& pt) override;
  void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::PoseTwist& pt,
    const dh_std::Range<double>& dead_zone);

private:
  // rosparams
  double max_hor_pos_err_;  // [m]
  double max_ver_pos_err_;  // [m]
  double max_hor_vel_;      // [m/s]
  double max_ver_vel_;      // [m/s]
  double max_attitude_;     // [rad]
  double max_yawrate_;      // [rad/s]
  double max_yaw_err_;      // [rad]

  // Constant
  KDL::Vector max_pos_err_;

  // Mutable
  ros::Time t_last_rcin_;
  KDL::Vector tar_vel_;
  KDL::Vector tar_pos_;
  KDL::Euler tar_rpy_;

  // Publisher
  ros::Publisher cmd_pub_;

  void getRosParams(ros::NodeHandle& pnh);
  void registerPublishers(ros::NodeHandle& nh);
};
}  // namespace tobas_rc_teleop
