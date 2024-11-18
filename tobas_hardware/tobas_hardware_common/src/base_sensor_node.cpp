#include "../include/tobas_hardware_common/base_sensor_node.hpp"

using namespace std;

namespace hardware
{
BaseSensorNode::BaseSensorNode(const std::string& name, const rclcpp::NodeOptions& options) : super(name, options)
{
  start_ss_ = createService<Empty>(name + kStartMainTimerSrvSuffix, &self::startMainTimerSrvCb, this);
  stop_ss_ = createService<Empty>(name + kStopMainTimerSrvSuffix, &self::stopMainTimerSrvCb, this);
}

void BaseSensorNode::startMainTimerSrvCb(const Empty::Request::ConstSharedPtr&, const Empty::Response::SharedPtr&)
{
  main_timer_->reset();
}

void BaseSensorNode::stopMainTimerSrvCb(const Empty::Request::ConstSharedPtr&, const Empty::Response::SharedPtr&)
{
  main_timer_->cancel();
}
}  // namespace hardware
