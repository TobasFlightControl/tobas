#include <rclcpp/rclcpp.hpp>

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
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  // PubSub
  rclcpp::Publisher setpoint_pos_local_pub_;
  rclcpp::Subscriber pos_yaw_sub_;

  void positionYawCb(const tobas_msgs::PositionYawConstPtr& tbs);
};
}  // namespace tobas_mavros_bridge
