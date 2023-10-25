#pragma once

#include <Eigen/Core>

#include <dh_std_tools/range.hpp>
#include <dh_std_tools/first_order_filter.hpp>

#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RCInput.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccYawController : public BaseController
{
  static constexpr double kDefaultDelayTimeConst = 0.;  // [s] 応答が悪くなるからデフォルトは遅延0

  using super = BaseController;

public:
  explicit PosVelAccYawController();

  void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) override;
  void reset(const tobas_msgs::PoseTwist& pt) override;
  void update(const tobas_msgs::RCInput& rcin, const dh_std::Range<double>& dead_zone);

private:
  ros::Time t_last_rcin_;
  dh_std::FirstOrderFilter<Eigen::Vector3d> vel_filter_;
  Eigen::Vector3d tar_vel_;
  Eigen::Vector3d tar_pos_;
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
