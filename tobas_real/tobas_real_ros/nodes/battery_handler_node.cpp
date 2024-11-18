#include <tobas_property_tree/property_tree.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/adc.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_real_msgs/srv/set_battery_params.hpp>

using namespace std;
using namespace real::handler::adc;
namespace fs = filesystem;

class BatteryHandlerNode : public tobas::BaseNode
{
  static constexpr double kMinVoltageThresh = 3.;  // [V]

  using self = BatteryHandlerNode;
  using super = tobas::BaseNode;
  using SetParams = tobas_real_msgs::srv::SetBatteryParams;

public:
  explicit BatteryHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  double voltage_coef_;
  double current_coef_;

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Adc> adc_sub_;
  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  bool getConfig();
  void registerPubSub();

  void adcCb(const tobas_msgs::msg::Adc::ConstSharedPtr& adc);
  void setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res);
};

BatteryHandlerNode::BatteryHandlerNode(const rclcpp::NodeOptions& options) : super("battery_handler", options)
{
  if (!pt_.initialize((fs::path(real::kTobasResourceDir) / get_name()).replace_extension(".ini")))
  {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  set_params_ss_ = createService<SetParams>(kSetParamSrv, &self::setParamsCb, this);

  if (!getConfig())
  {
    TOBAS_ERROR("Failed to get configurations. This node will not work until they are set.");
    return;
  }

  registerPubSub();
}

bool BatteryHandlerNode::getConfig()
{
  if (!pt_.get(kVoltageKey, voltage_coef_))
  {
    TOBAS_ERROR("Failed to get \"", kVoltageKey, "\".");
    return false;
  }

  if (!pt_.get(kCurrentKey, current_coef_))
  {
    TOBAS_ERROR("Failed to get \"", kCurrentKey, "\".");
    return false;
  }

  return true;
}

void BatteryHandlerNode::registerPubSub()
{
  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  adc_sub_ = createSubscriber(real::kADCTopic, &self::adcCb, this);
}

void BatteryHandlerNode::adcCb(const tobas_msgs::msg::Adc::ConstSharedPtr& adc)
{
  // Create battery message
  auto battery_msg = std::make_unique<tobas_msgs::msg::Battery>();
  battery_msg->header = adc->header;

  // Fill values
  battery_msg->voltage = adc->voltage * voltage_coef_;
  battery_msg->current = adc->current * current_coef_;

  // Check voltage
  if (battery_msg->voltage < kMinVoltageThresh)
  {
    TOBAS_WARN_THROTTLE(
      tobas::kTypicalWarnPeriod, "Battery voltage is too low (", battery_msg->voltage,
      " V). Battery message is not published.");
    return;
  }

  // Publish battery message
  battery_pub_->publish(move(battery_msg));
}

void BatteryHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& req,
  const SetParams::Response::SharedPtr& res)
{
  // Verify parameters
  if (req->voltage_coef <= 0.)
  {
    res->success = false;
    res->message = "Voltage coefficient must be positive.";
    return;
  }
  if (req->current_coef <= 0.)
  {
    res->success = false;
    res->message = "Current coefficient must be positive.";
    return;
  }

  // Update parameters
  voltage_coef_ = req->voltage_coef;
  current_coef_ = req->current_coef;

  // Save parameters
  pt_.set(kVoltageKey, req->voltage_coef);
  pt_.set(kCurrentKey, req->current_coef);
  if (!pt_.save())
  {
    res->success = false;
    res->message = "Failed to save parameters.";
    return;
  }

  if (battery_pub_ == nullptr)
    registerPubSub();

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryHandlerNode)
