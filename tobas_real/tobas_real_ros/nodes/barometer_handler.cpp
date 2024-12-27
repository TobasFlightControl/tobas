#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/fluid_pressure_stamped.hpp>

using namespace std;

class BarometerHandlerNode : public tobas::BaseNode
{
  using self = BarometerHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit BarometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::FluidPressureStamped> pres_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressureStamped> pres_sub_;

  void airPressureCb(const tobas_msgs::msg::FluidPressureStamped::ConstSharedPtr& pres_in);
};

BarometerHandlerNode::BarometerHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_barometer_handler", options)
{
  pres_pub_ = createPublisher<tobas_msgs::msg::FluidPressureStamped>(tobas::kAirPressureRawTopic);
  pres_sub_ = createSubscriber(real::kAirPressureTopic, &self::airPressureCb, this);
}

void BarometerHandlerNode::airPressureCb(const tobas_msgs::msg::FluidPressureStamped::ConstSharedPtr& pres_in)
{
  auto pres_out = std::make_unique<tobas_msgs::msg::FluidPressureStamped>(*pres_in);
  pres_pub_->publish(move(pres_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BarometerHandlerNode)
