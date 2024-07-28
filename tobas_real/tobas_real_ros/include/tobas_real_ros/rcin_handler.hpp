#pragma once

#include <array>
#include <std_srvs/Trigger.h>

#include <tobas_std_tools/range.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_hal_msgs/Sbus.h>

namespace tobas_real_ros
{
class RCInputHandler : public tobas::BaseNode
{
  using self = RCInputHandler;
  using super = tobas::BaseNode;

public:
  explicit RCInputHandler(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // Config
  tobas_std::Range<uint16_t> roll_range_;
  tobas_std::Range<uint16_t> pitch_range_;
  tobas_std::Range<uint16_t> yaw_range_;
  tobas_std::Range<uint16_t> throttle_range_;
  std::array<uint16_t, tobas::kNumFlightModes> modes_;
  uint16_t estop_on_, estop_off_;
  uint16_t gpsw_on_, gpsw_off_;

  ptree::PropertyClient property_client_;

  ros::Publisher rcin_pub_;
  ros::Subscriber sbus_sub_;
  ros::ServiceServer reload_config_srv_;

  void setToDefaults();
  bool reloadConfig();

  void sbusCb(const tobas_hal_msgs::SbusConstPtr& sbus);
  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
};
}  // namespace tobas_real_ros
