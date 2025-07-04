#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_real_common/constants.hpp>

#include <tobas_msgs/msg/fluid_pressure.hpp>

class BarometerHandlerNode : public tobas::BaseNode
{
  using self = BarometerHandlerNode;
  using super = tobas::BaseNode;

  static constexpr size_t kNotUpdatedCountThresh = 2;

public:
  explicit BarometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t not_updated_cnt_ = 0;
  tobas_msgs::msg::FluidPressure::ConstSharedPtr prev_pres_;

  ros2::PublisherPtr<tobas_msgs::msg::FluidPressure> pres_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressure> pres_sub_;

  void airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pres_in);
};

BarometerHandlerNode::BarometerHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_barometer_handler", options)
{
  pres_pub_ = createPublisher<tobas_msgs::msg::FluidPressure>(tobas::kAirPressureTopic);
  pres_sub_ = createSubscriber(real::kAirPressureTopic, &self::airPressureCb, this);
}

void BarometerHandlerNode::airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pres_in)
{
  // First message
  if (!prev_pres_) {
    prev_pres_ = pres_in;
    return;
  }

  // Verify that the sensor data is updated
  // 本当に値が変わっていないだけの可能性もあるため，連続して更新されなかった場合のみ警告する．
  if (pres_in->pressure == prev_pres_->pressure) {
    ++not_updated_cnt_;
    if (not_updated_cnt_ >= kNotUpdatedCountThresh) {
      TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Barometer data has not been updated—skipping message.");
      not_updated_cnt_ = 0;
      return;
    }
  }
  else {
    not_updated_cnt_ = 0;
  }

  // Update the latest data
  prev_pres_ = pres_in;

  // Publish a calibrated data
  auto pres_out = std::make_unique<tobas_msgs::msg::FluidPressure>(*pres_in);
  pres_pub_->publish(move(pres_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BarometerHandlerNode)
