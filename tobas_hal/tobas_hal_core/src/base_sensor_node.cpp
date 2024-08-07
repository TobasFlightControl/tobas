#include <tobas_constants/constants.hpp>

#include "../include/tobas_hal_core/base_sensor_node.hpp"

using namespace std;

namespace hal
{
BaseSensorNode::BaseSensorNode(, const string& name) : super(node, pnh, name)
{
  start_ss_ = node_.advertiseService(name + tobas::kStartMainTimerSrvSuffix, &self::startMainTimerSrvCb, this);
  stop_ss_ = node_.advertiseService(name + tobas::kStopMainTimerSrvSuffix, &self::stopMainTimerSrvCb, this);
}

bool BaseSensorNode::startMainTimerSrvCb(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
  main_timer_.start();
  return true;
}

bool BaseSensorNode::stopMainTimerSrvCb(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
  main_timer_.stop();
  return true;
}
}  // namespace hal
