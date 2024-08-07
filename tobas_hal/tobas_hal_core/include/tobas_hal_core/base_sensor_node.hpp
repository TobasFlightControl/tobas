#pragma once

#include <ros/timer.h>
#include <std_srvs/Empty.h>

#include <tobas_node/node.hpp>

namespace hal
{
class BaseSensorNode : public tobas::BaseNode
{
  using self = BaseSensorNode;
  using super = tobas::BaseNode;

public:
  explicit BaseSensorNode(, const std::string& name);

protected:
  rclcpp::Timer main_timer_;

private:
  rclcpp::ServiceServer start_ss_;
  rclcpp::ServiceServer stop_ss_;

  bool startMainTimerSrvCb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  bool stopMainTimerSrvCb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
};
}  // namespace hal
