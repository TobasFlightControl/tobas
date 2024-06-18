#include <tobas_tools/constants.hpp>

#include "../include/tobas_navio_ros/base_sensor_node.hpp"

namespace tobas_navio_ros
{
BaseSensorNode::BaseSensorNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& name)
  : super(nh, pnh, name)
{
  start_ss_ = nh_.advertiseService(name + tobas::kStartMainTimerSrvSuffix, &self::startMainTimerSrvCb, this);
  stop_ss_ = nh_.advertiseService(name + tobas::kStopMainTimerSrvSuffix, &self::stopMainTimerSrvCb, this);
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
}  // namespace tobas_navio_ros
