#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/gnss.hpp>

class GnssStateCheckerNode : public tobas::BaseNode
{
  using self = GnssStateCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit GnssStateCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;

  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};

GnssStateCheckerNode::GnssStateCheckerNode(const rclcpp::NodeOptions& options) : super("gnss_state_checker", options)
{
  gnss_sub_ = createSubscriber<tobas_msgs::Gnss>(tobas::topic::kGnss, &self::gnssCb, this);
}

void GnssStateCheckerNode::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  if (gnss->fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
    TOBAS_INFO_THROTTLE(1., "GNSS 3D Fix");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(GnssStateCheckerNode)
