#pragma once

#include <tobas_std_tools/first_order_filter.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccYawController : public BaseController
{
  static constexpr double kDefaultDelayTimeConst = 0.;  // [s] 応答が悪くなるからデフォルトは遅延0

  using super = BaseController;

public:
  explicit PosVelAccYawController(const tobas::Drone& drone);

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::Odometry& odom,
    const double& battery_voltage) override;

private:
  bool is_up_commanded_ = false;
  ros::Time t_last_rcin_;
  tobas_std::FirstOrderFilter<KDL::Vector> vel_filter_;
  KDL::Vector tar_vel_F_;
  KDL::Vector tar_pos_;
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;       // [m/s]
  double max_ver_vel_;       // [m/s]
  double max_yawrate_;       // [rad/s]
  double delay_time_const_;  // [s]

  // Publisher
  ros::Publisher cmd_pub_;

  void getRosParams(ros::NodeHandle& pnh);
  void registerPublishers(ros::NodeHandle& nh);
};
}  // namespace tobas_rc_teleop
