#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>
#include <tobas_msgs/msg/rc_input.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;
using namespace real::handler::rcin;

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
  tobas_std::Range<uint16_t> throt_range_;
  std::array<uint16_t, tobas::kNumFlightModes> modes_;
  uint16_t estop_on_, estop_off_;
  uint16_t gpsw_on_, gpsw_off_;

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::msg::RCInput> rcin_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::msg::Sbus> sbus_sub_;

  void readConfig();
  void setToDefaults();

  bool paramsCb(const std::vector<double>& params);
  void sbusCb(const tobas_hal_msgs::msg::Sbus::ConstSharedPtr& sbus);
};

RCInputHandlerNode::RCInputHandlerNode(const rclcpp::NodeOptions& options) : super("rcin_handler", options)
{
  if (!pt_.initialize(linux::expandUser(kIniPath)))
    TOBAS_EXIT("Failed to initialize property tree.");

  readConfig();

  addDynamicDoubleArrayParam(real::handler::kParamName, &self::paramsCb, this);

  rcin_pub_ = createPublisher<tobas_msgs::msg::RCInput>(tobas::kRcInputTopic);
  sbus_sub_ = createSubscriber(hal::kSbusTopic, &self::sbusCb, this);
}

void RCInputHandlerNode::readConfig()
{
  if (pt_.get(kRollLeftKey, roll_range_.lower) < 0)
  {
    TOBAS_WARN("Failed to get \"", kRollLeftKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kRollRightKey, roll_range_.upper) < 0)
  {
    TOBAS_WARN("Failed to get \"", kRollRightKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }

  if (pt_.get(kPitchDownKey, pitch_range_.lower) < 0)
  {
    TOBAS_WARN("Failed to get \"", kPitchDownKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kPitchUpKey, pitch_range_.upper) < 0)
  {
    TOBAS_WARN("Failed to get \"", kPitchUpKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }

  if (pt_.get(kYawRightKey, yaw_range_.lower) < 0)
  {
    TOBAS_WARN("Failed to get \"", kYawRightKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kYawLeftKey, yaw_range_.upper) < 0)
  {
    TOBAS_WARN("Failed to get \"", kYawLeftKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }

  if (pt_.get(kThrotDownKey, throt_range_.lower) < 0)
  {
    TOBAS_WARN("Failed to get \"", kThrotDownKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kThrotUpKey, throt_range_.upper) < 0)
  {
    TOBAS_WARN("Failed to get \"", kThrotUpKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }

  if (pt_.get(kModeProgramKey, modes_.at(tobas::kFlightModeProgram)) < 0)
  {
    TOBAS_WARN("Failed to get \"", kModeProgramKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kModeStabilizeKey, modes_.at(tobas::kFlightModeStabilize)) < 0)
  {
    TOBAS_WARN("Failed to get \"", kModeStabilizeKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kModeAcrobatKey, modes_.at(tobas::kFlightModeAcrobat)) < 0)
  {
    TOBAS_WARN("Failed to get \"", kModeAcrobatKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }

  if (pt_.get(kEStopOnKey, estop_on_) < 0)
  {
    TOBAS_WARN("Failed to get \"", kEStopOnKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kEStopOffKey, estop_off_) < 0)
  {
    TOBAS_WARN("Failed to get \"", kEStopOffKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }

  if (pt_.get(kGPSwOnKey, gpsw_on_) < 0)
  {
    TOBAS_WARN("Failed to get \"", kGPSwOnKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
  if (pt_.get(kGPSwOffKey, gpsw_off_) < 0)
  {
    TOBAS_WARN("Failed to get \"", kGPSwOffKey, "\". from configuration file. All params are set to defaults.");
    setToDefaults();
    return;
  }
}

void RCInputHandlerNode::setToDefaults()
{
  roll_range_.set(tobas::kPwmMin, tobas::kPwmMax);
  pitch_range_.set(tobas::kPwmMax, tobas::kPwmMin);
  yaw_range_.set(tobas::kPwmMax, tobas::kPwmMin);
  throt_range_.set(tobas::kPwmMax, tobas::kPwmMin);

  modes_[tobas::kFlightModeProgram] = tobas::kPwmMin;
  modes_[tobas::kFlightModeStabilize] = tobas::kPwmMid;
  modes_[tobas::kFlightModeAcrobat] = tobas::kPwmMax;

  estop_on_ = tobas::kPwmMin;
  estop_off_ = tobas::kPwmMax;
  gpsw_on_ = tobas::kPwmMin;
  gpsw_off_ = tobas::kPwmMax;
}

bool RCInputHandlerNode::paramsCb(const std::vector<double>& params)
{
  // Skip first call
  if (params.size() == 0)
    return false;

  // Check size
  if (params.size() != kParamSize)
  {
    TOBAS_ERROR("Parameter size mismatch.");
    return false;
  }

  // Update parameters
  roll_range_.lower = params.at(kRollLeftChannel);
  roll_range_.upper = params.at(kRollRightChannel);
  pitch_range_.lower = params.at(kPitchDownChannel);
  pitch_range_.upper = params.at(kPitchUpChannel);
  yaw_range_.lower = params.at(kYawRightChannel);
  yaw_range_.upper = params.at(kYawLeftChannel);
  throt_range_.lower = params.at(kThrotDownChannel);
  throt_range_.upper = params.at(kThrotUpChannel);
  modes_.at(tobas::kFlightModeProgram) = params.at(kModeProgramChannel);
  modes_.at(tobas::kFlightModeStabilize) = params.at(kModeStabilizeChannel);
  modes_.at(tobas::kFlightModeAcrobat) = params.at(kModeAcrobatChannel);
  estop_on_ = params.at(kEStopOnChannel);
  estop_off_ = params.at(kEStopOffChannel);
  gpsw_on_ = params.at(kGPSwOnChannel);
  gpsw_off_ = params.at(kGPSwOffChannel);

  // Save parameters
  pt_.set(kRollLeftKey, params.at(kRollLeftChannel));
  pt_.set(kRollRightKey, params.at(kRollRightChannel));
  pt_.set(kPitchDownKey, params.at(kPitchDownChannel));
  pt_.set(kPitchUpKey, params.at(kPitchUpChannel));
  pt_.set(kYawRightKey, params.at(kYawRightChannel));
  pt_.set(kYawLeftKey, params.at(kYawLeftChannel));
  pt_.set(kThrotDownKey, params.at(kThrotDownChannel));
  pt_.set(kThrotUpKey, params.at(kThrotUpChannel));
  pt_.set(kModeProgramKey, params.at(kModeProgramChannel));
  pt_.set(kModeStabilizeKey, params.at(kModeStabilizeChannel));
  pt_.set(kModeAcrobatKey, params.at(kModeAcrobatChannel));
  pt_.set(kEStopOnKey, params.at(kEStopOnChannel));
  pt_.set(kEStopOffKey, params.at(kEStopOffChannel));
  pt_.set(kGPSwOnKey, params.at(kGPSwOnChannel));
  pt_.set(kGPSwOffKey, params.at(kGPSwOffChannel));
  if (pt_.save())
  {
    TOBAS_ERROR("Failed to save parameters.");
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
    sbus->data[real::kRcChannelThrottle], throt_range_.lower, throt_range_.upper, tobas::kRCInputMin,
    tobas::kRCInputMax);
  rcin_msg->mode = tobas_std::closestIndex(modes_, sbus->data[real::kRcChannelMode]);
  rcin_msg->e_stop =
    abs(sbus->data[real::kRcChannelEStop] - estop_on_) < abs(sbus->data[real::kRcChannelEStop] - estop_off_);
  rcin_msg->gpsw = abs(sbus->data[real::kRcChannelGPSw] - gpsw_on_) < abs(sbus->data[real::kRcChannelGPSw] - gpsw_off_);

  // Publish message
  rcin_pub_->publish(move(rcin_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(RCInputHandlerNode)
