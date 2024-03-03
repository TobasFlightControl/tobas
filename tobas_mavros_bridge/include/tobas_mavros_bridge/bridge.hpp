#include <ros/ros.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PositionYaw.h>

namespace tobas_mavros_bridge
{
class TobasMavrosBridge : public tobas::BaseNode
{
  using self = TobasMavrosBridge;
  using super = tobas::BaseNode;

public:
  explicit TobasMavrosBridge(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // PubSub
  ros::Publisher setpoint_pos_local_pub_;
  ros::Subscriber pos_yaw_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void positionYawCb(const tobas_msgs::PositionYawConstPtr& tbs);
};
}  // namespace tobas_mavros_bridge
