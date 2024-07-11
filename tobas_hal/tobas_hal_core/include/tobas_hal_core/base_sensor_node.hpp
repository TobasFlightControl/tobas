#pragma once

#include <ros/timer.h>
#include <std_srvs/Empty.h>

#include <tobas_tools/node.hpp>

namespace hal
{
class BaseSensorNode : public tobas::BaseNode
{
  using self = BaseSensorNode;
  using super = tobas::BaseNode;

public:
  explicit BaseSensorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name);

protected:
  ros::Timer main_timer_;

private:
  ros::ServiceServer start_ss_;
  ros::ServiceServer stop_ss_;

  bool startMainTimerSrvCb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  bool stopMainTimerSrvCb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
};
}  // namespace hal
