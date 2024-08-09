#pragma once

#include <array>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_std_tools/range.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_msgs/Sbus.h>

namespace tobas_real_ros
{
class RCInputHandler : public tobas::BaseNode
{
  using self = RCInputHandler;
  using super = tobas::BaseNode;

public:
  explicit RCInputHandler(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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

  PublisherPtr<> rcin_pub_;
  SubscriberPtr<> sbus_sub_;
  rclcpp::ServiceServer reload_config_srv_;

  void setToDefaults();
  bool reloadConfig();

  void sbusCb(const tobas_hal_msgs::Sbus::ConstSharedPtr& sbus);
  bool reloadConfigCb(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
};
}  // namespace tobas_real_ros
