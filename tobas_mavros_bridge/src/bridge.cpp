#include <geometry_msgs/PoseStamped.h>

#include <dh_kdl/quaternion.hpp>
#include <dh_kdl/conversion/kdl_msg.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_mavros_bridge/bridge.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mavros_bridge
{
TobasMavrosBridge::TobasMavrosBridge(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  registerPublishers();
  registerSubscribers();
}

void TobasMavrosBridge::getRosParams()
{
}

void TobasMavrosBridge::registerPublishers()
{
  setpoint_pos_local_pub_ =
    nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
}

void TobasMavrosBridge::registerSubscribers()
{
  super::registerSubscribers();

  pos_yaw_sub_ =
    nh_.subscribe(tobas::kPositionYawCmdTopic, 1, &self::positionYawCb, this, tcpNoDelay());
}

void TobasMavrosBridge::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void TobasMavrosBridge::positionYawCb(const tobas_msgs::PositionYawConstPtr& tbs)
{
  const auto mav = boost::make_shared<geometry_msgs::PoseStamped>();

  // NWU -> ENU
  mav->pose.position.x = -tbs->pos.y();
  mav->pose.position.y = tbs->pos.x();
  mav->pose.position.z = tbs->pos.z();

  const auto quat = Quaternion::RPY(0, 0, tbs->yaw + M_PI_2);
  quaternionKDLToMsg(quat, mav->pose.orientation);

  setpoint_pos_local_pub_.publish(mav);
}
}  // namespace tobas_mavros_bridge
