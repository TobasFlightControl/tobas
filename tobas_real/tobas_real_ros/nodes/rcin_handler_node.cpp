#include <array>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_client/property_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>
#include <tobas_msgs/msg/rc_input.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;

class RCInputHandlerNode : public tobas::BaseNode
{
  using self = RCInputHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit RCInputHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  tobas_std::Range<uint16_t> roll_range_;
  tobas_std::Range<uint16_t> pitch_range_;
  tobas_std::Range<uint16_t> yaw_range_;
  tobas_std::Range<uint16_t> throttle_range_;
  std::array<uint16_t, tobas::kNumFlightModes> modes_;
  uint16_t estop_on_, estop_off_;
  uint16_t gpsw_on_, gpsw_off_;

  ptree::PropertyClient::SharedPtr property_client_;

  ros2::PublisherPtr<tobas_msgs::msg::RCInput> rcin_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::msg::Sbus> sbus_sub_;
  ros2::ServiceServerPtr<std_srvs::srv::Trigger> reload_config_srv_;

  ros2::TimerPtr initialize_timer_;
  void initializeTimerCb();

  void setToDefaults();
  bool reloadConfig();

  void sbusCb(const tobas_hal_msgs::msg::Sbus::ConstSharedPtr& sbus);
  void reloadConfigCb(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
    const std_srvs::srv::Trigger::Response::SharedPtr& res);
};

RCInputHandlerNode::RCInputHandlerNode(const rclcpp::NodeOptions& options) : super("rcin_handler", options)
{
  initialize_timer_ = createTimer(0ns, &self::initializeTimerCb, this);
}

void RCInputHandlerNode::initializeTimerCb()
{
  property_client_ = std::make_shared<ptree::PropertyClient>(shared_from_this(), real::kPropertyServerFC);
  reloadConfig();

  rcin_pub_ = createPublisher<tobas_msgs::msg::RCInput>(tobas::kRcInputTopic);
  sbus_sub_ = createSubscriber(hal::kSbusTopic, &self::sbusCb, this);

  reload_config_srv_ =
    createService<std_srvs::srv::Trigger>(name() + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);

  initialize_timer_->cancel();
}

void RCInputHandlerNode::setToDefaults()
{
  roll_range_.set(tobas::kPwmMin, tobas::kPwmMax);
  pitch_range_.set(tobas::kPwmMax, tobas::kPwmMin);
  yaw_range_.set(tobas::kPwmMax, tobas::kPwmMin);
  throttle_range_.set(tobas::kPwmMax, tobas::kPwmMin);

  modes_[tobas::kFlightModeProgram] = tobas::kPwmMin;
  modes_[tobas::kFlightModeStabilize] = tobas::kPwmMid;
  modes_[tobas::kFlightModeAcrobat] = tobas::kPwmMax;

  estop_on_ = tobas::kPwmMin;
  estop_off_ = tobas::kPwmMax;
  gpsw_on_ = tobas::kPwmMin;
  gpsw_off_ = tobas::kPwmMax;
}

bool RCInputHandlerNode::reloadConfig()
{
  if (property_client_->get(real::kConfigKey_RcRollLeft, roll_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcRollRight, roll_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_->get(real::kConfigKey_RcPitchDown, pitch_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcPitchUp, pitch_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_->get(real::kConfigKey_RcYawRight, yaw_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcYawLeft, yaw_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_->get(real::kConfigKey_RcThrottleDown, throttle_range_.lower) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcThrottleUp, throttle_range_.upper) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_->get(real::kConfigKey_RcModeProgram, modes_[tobas::kFlightModeProgram]) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcModeStabilize, modes_[tobas::kFlightModeStabilize]) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcModeAcrobat, modes_[tobas::kFlightModeAcrobat]) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_->get(real::kConfigKey_RcEStopOn, estop_on_) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcEStopOff, estop_off_) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }

  if (property_client_->get(real::kConfigKey_RcGPSwOn, gpsw_on_) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }
  if (property_client_->get(real::kConfigKey_RcGPSwOff, gpsw_off_) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    setToDefaults();
    return false;
  }

  return true;
}

void RCInputHandlerNode::sbusCb(const tobas_hal_msgs::msg::Sbus::ConstSharedPtr& sbus)
{
  // Create message
  auto rcin_msg = std::make_unique<tobas_msgs::msg::RCInput>();

  // Fill header
  rcin_msg->header = sbus->header;

  // Fill duty periods for each channel
  rcin_msg->roll = math::remap<double>(
    sbus->data[real::kRcChannelRoll], roll_range_.lower, roll_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);
  rcin_msg->pitch = math::remap<double>(
    sbus->data[real::kRcChannelPitch], pitch_range_.lower, pitch_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);
  rcin_msg->yaw = math::remap<double>(
    sbus->data[real::kRcChannelYaw], yaw_range_.lower, yaw_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);
  rcin_msg->throttle = math::remap<double>(
    sbus->data[real::kRcChannelThrottle], throttle_range_.lower, throttle_range_.upper, tobas::kRCInputMin,
    tobas::kRCInputMax);
  rcin_msg->mode = tobas_std::closestIndex(modes_, sbus->data[real::kRcChannelMode]);
  rcin_msg->e_stop =
    abs(sbus->data[real::kRcChannelEStop] - estop_on_) < abs(sbus->data[real::kRcChannelEStop] - estop_off_);
  rcin_msg->gpsw = abs(sbus->data[real::kRcChannelGPSw] - gpsw_on_) < abs(sbus->data[real::kRcChannelGPSw] - gpsw_off_);

  // Publish message
  rcin_pub_->publish(move(rcin_msg));
}

void RCInputHandlerNode::reloadConfigCb(
  const std_srvs::srv::Trigger::Request::ConstSharedPtr&,
  const std_srvs::srv::Trigger::Response::SharedPtr& res)
{
  if (!reloadConfig())
  {
    res->success = false;
    res->message = "Failed to reload configurations.";
    return;
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(RCInputHandlerNode)
