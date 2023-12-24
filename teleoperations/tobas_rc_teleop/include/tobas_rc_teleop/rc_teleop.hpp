#pragma once

#include <memory>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/RCInput.h>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RCTeleop : public tobas::BaseNode
{
  using self = RCTeleop;
  using super = tobas::BaseNode;

public:
  explicit RCTeleop(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  enum stage_t
  {
    CHECK_PREREQUISITES,
    WAIT_FOR_ESTOP,
    ESTOP_ON,
    FIRST_COMMAND,
    RUNNING,
  };

  // rosparams
  double dead_zone_rate_;
  std::vector<std::string> mode_names_;

  // Constants
  tobas_std::Range<double> dead_zone_;

  // Mutables
  stage_t stage_ = CHECK_PREREQUISITES;
  uint8_t last_mode_;
  tobas_msgs::OdometryConstPtr odom_;
  tobas_msgs::BatteryConstPtr battery_;

  // Controllers
  std::vector<std::unique_ptr<BaseController>> controllers_;

  // PubSub
  ros::Publisher event_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber rcin_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void rcInputCb(const tobas_msgs::RCInputConstPtr& rcin);
};
}  // namespace tobas_rc_teleop
