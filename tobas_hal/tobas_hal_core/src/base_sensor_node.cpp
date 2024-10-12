#include <tobas_constants/constants.hpp>

#include "../include/tobas_hal_core/base_sensor_node.hpp"

using namespace std;

namespace hal
{
BaseSensorNode::BaseSensorNode(const std::string& name, const rclcpp::NodeOptions& options) : super(name, options)
{
  start_ss_ = createService<Empty>(name + tobas::kStartMainTimerSrvSuffix, &self::startMainTimerSrvCb, this);
  stop_ss_ = createService<Empty>(name + tobas::kStopMainTimerSrvSuffix, &self::stopMainTimerSrvCb, this);
}

void BaseSensorNode::startMainTimerSrvCb(const Empty::Request::ConstSharedPtr&, const Empty::Response::SharedPtr&)
{
  main_timer_->reset();
}

void BaseSensorNode::stopMainTimerSrvCb(const Empty::Request::ConstSharedPtr&, const Empty::Response::SharedPtr&)
{
  main_timer_->cancel();
}
}  // namespace hal
