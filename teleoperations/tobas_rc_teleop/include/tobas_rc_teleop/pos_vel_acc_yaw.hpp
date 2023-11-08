#pragma once

#include <Eigen/Core>

#include <dh_std_tools/range.hpp>
#include <dh_std_tools/first_order_filter.hpp>

#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/RCInput.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccYawController : public BaseController
{
  static constexpr double kDefaultDelayTimeConst = 0.;  // [s] 応答が悪くなるからデフォルトは遅延0

  using super = BaseController;

public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::Odometry& odom,
    const double& battery_voltage,
    const dh_std::Range<double>& dead_zone) override;

private:
  ros::Time t_last_rcin_;
  dh_std::FirstOrderFilter<KDL::Vector> vel_filter_;
  KDL::Vector tar_vel_;
  KDL::Vector tar_pos_;
  double tar_yaw_;
  KDL::Vector max_pos_err_;

  // rosparams
  double max_hor_pos_err_;   // [m]
  double max_ver_pos_err_;   // [m]
  double max_hor_vel_;       // [m/s]
  double max_ver_vel_;       // [m/s]
  double max_yawrate_;       // [rad/s]
  double max_yaw_err_;       // [rad]
  double delay_time_const_;  // [s]

  // Publisher
  ros::Publisher cmd_pub_;

  void getRosParams(ros::NodeHandle& pnh);
  void registerPublishers(ros::NodeHandle& nh);
};
}  // namespace tobas_rc_teleop
