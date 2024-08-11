#pragma once

#include <ros/timer.h>
#include <std_srvs/srv/empty.hpp>

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
  ServicePtr<> start_ss_;
  ServicePtr<> stop_ss_;

  bool startMainTimerSrvCb(const std_srvs::srv::Empty::Request::ConstSharedPtr& req, const std_srvs::srv::Empty::Response::SharedPtr& res);
  bool stopMainTimerSrvCb(const std_srvs::srv::Empty::Request::ConstSharedPtr& req, const std_srvs::srv::Empty::Response::SharedPtr& res);
};
}  // namespace hal
