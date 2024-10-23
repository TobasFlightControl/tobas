#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/battery.hpp>

class BatteryLPFNode : public tobas::BaseNode
{
  // LPFのカットオフ周波数 [Hz]
  // 小さすぎると離陸時の急激な電圧降下に追従できず，所望の推力が出ない．
  // ADCのノイズを軽減できる最大限の値に設定すべき．
  static constexpr double kLpfCutoff = 1.;

  using self = BatteryLPFNode;
  using super = tobas::BaseNode;

public:
  explicit BatteryLPFNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  dsp::LowPassFilter<double> voltage_lpf_, current_lpf_;
  tobas_msgs::msg::Battery::ConstSharedPtr last_msg_;

  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_lpf_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_raw_sub_;

  void batteryRawCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery_raw);
};

using namespace std;

BatteryLPFNode::BatteryLPFNode(const rclcpp::NodeOptions& options) : super("battery_lpf", options)
{
  battery_lpf_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryLpfTopic);
  battery_raw_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryRawCb, this);
}

void BatteryLPFNode::batteryRawCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery_raw)
{
  if (last_msg_ == nullptr)
  {
    TOBAS_INFO("First raw battery message is received.");
    voltage_lpf_.initialize(kLpfCutoff, battery_raw->voltage);
    current_lpf_.initialize(kLpfCutoff, battery_raw->current);
    last_msg_ = battery_raw;
    return;
  }

  const auto dt = (battery_raw->header.stamp - last_msg_->header.stamp).seconds();
  last_msg_ = battery_raw;

  if (voltage_lpf_.update(battery_raw->voltage, dt) < 0)
    TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Failed to update voltage LPF: ", voltage_lpf_.errorMessage());
  if (current_lpf_.update(battery_raw->current, dt) < 0)
    TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Failed to update current LPF: ", current_lpf_.errorMessage());

  auto battery_filtered = std::make_unique<tobas_msgs::msg::Battery>(*battery_raw);
  battery_filtered->voltage = voltage_lpf_.getOutput();
  battery_filtered->current = current_lpf_.getOutput();
  battery_lpf_pub_->publish(move(battery_filtered));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryLPFNode)
