#include <geometry_msgs/msg/PoseStamped.h>

#include <tobas_kdl/quaternion.hpp>
#include <tobas_kdl_msgs/conversion/kdl_msg.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_mavros_bridge/bridge.hpp"

using namespace std;

namespace tobas_mavros_bridge
{
TobasMavrosBridge::TobasMavrosBridge(, const string& name)
  : super(node, pnh, name)
{
  setpoint_pos_local_pub_ = node_.advertise<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 1);
  pos_yaw_sub_ = node_.subscribe(tobas::kPositionYawCmdTopic, 1, &self::positionYawCb, this, tcpNoDelay());
}

void TobasMavrosBridge::positionYawCb(const tobas_msgs::PositionYawConstPtr& tbs)
{
  const auto mav = boost::make_shared<geometry_msgs::msg::PoseStamped>();

  // NWU -> ENU
  mav->pose.position.x = -tbs->pos.y();
  mav->pose.position.y = tbs->pos.x();
  mav->pose.position.z = tbs->pos.z();

  const auto quat = kdl::Quaternion::RPY(0, 0, tbs->yaw + M_PI_2);
  quaternionKDLToMsg(quat, mav->pose.orientation);

  setpoint_pos_local_pub_.publish(mav);
}
}  // namespace tobas_mavros_bridge
