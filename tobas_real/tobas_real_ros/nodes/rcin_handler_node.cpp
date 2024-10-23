#include <tobas_math/core.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;
using namespace real::handler::rcin;
namespace fs = filesystem;

class RCInputHandlerNode : public tobas::BaseNode
{
  using self = RCInputHandlerNode;
  using super = tobas::BaseNode;
  using SetParams = tobas_real_msgs::srv::SetRCInputParams;

public:
  explicit RCInputHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  tobas_std::Range<uint16_t> roll_range_;
  tobas_std::Range<uint16_t> pitch_range_;
  tobas_std::Range<uint16_t> yaw_range_;
  tobas_std::Range<uint16_t> throt_range_;
  array<uint16_t, tobas::kNumFlightModes> modes_;
  uint16_t estop_on_, estop_off_;
  uint16_t gpsw_on_, gpsw_off_;

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::msg::RCInput> rcin_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::msg::Sbus> sbus_sub_;
  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  bool getConfig();
  void registerPubSub();

  void sbusCb(const tobas_hal_msgs::msg::Sbus::ConstSharedPtr& sbus);
  void setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res);
};

RCInputHandlerNode::RCInputHandlerNode(const rclcpp::NodeOptions& options) : super("rcin_handler", options)
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

bool RCInputHandlerNode::getConfig()
{
  if (!pt_.get(kRollLeftKey, roll_range_.lower))
  {
    TOBAS_ERROR("Failed to get \"", kRollLeftKey, "\".");
    return false;
  }
  if (!pt_.get(kRollRightKey, roll_range_.upper))
  {
    TOBAS_ERROR("Failed to get \"", kRollRightKey, "\".");
    return false;
  }

  if (!pt_.get(kPitchDownKey, pitch_range_.lower))
  {
    TOBAS_ERROR("Failed to get \"", kPitchDownKey, "\".");
    return false;
  }
  if (!pt_.get(kPitchUpKey, pitch_range_.upper))
  {
    TOBAS_ERROR("Failed to get \"", kPitchUpKey, "\".");
    return false;
  }

  if (!pt_.get(kYawRightKey, yaw_range_.lower))
  {
    TOBAS_ERROR("Failed to get \"", kYawRightKey, "\".");
    return false;
  }
  if (!pt_.get(kYawLeftKey, yaw_range_.upper))
  {
    TOBAS_ERROR("Failed to get \"", kYawLeftKey, "\".");
    return false;
  }

  if (!pt_.get(kThrotDownKey, throt_range_.lower))
  {
    TOBAS_ERROR("Failed to get \"", kThrotDownKey, "\".");
    return false;
  }
  if (!pt_.get(kThrotUpKey, throt_range_.upper))
  {
    TOBAS_ERROR("Failed to get \"", kThrotUpKey, "\".");
    return false;
  }

  if (!pt_.get(kModeProgramKey, modes_.at(tobas::flight_mode_t::PROGRAM_MODE)))
  {
    TOBAS_ERROR("Failed to get \"", kModeProgramKey, "\".");
    return false;
  }
  if (!pt_.get(kModeStabilizeKey, modes_.at(tobas::flight_mode_t::STABILIZE_MODE)))
  {
    TOBAS_ERROR("Failed to get \"", kModeStabilizeKey, "\".");
    return false;
  }
  if (!pt_.get(kModeAcrobatKey, modes_.at(tobas::flight_mode_t::ACROBAT_MODE)))
  {
    TOBAS_ERROR("Failed to get \"", kModeAcrobatKey, "\".");
    return false;
  }

  if (!pt_.get(kEStopOnKey, estop_on_))
  {
    TOBAS_ERROR("Failed to get \"", kEStopOnKey, "\".");
    return false;
  }
  if (!pt_.get(kEStopOffKey, estop_off_))
  {
    TOBAS_ERROR("Failed to get \"", kEStopOffKey, "\".");
    return false;
  }

  if (!pt_.get(kGPSwOnKey, gpsw_on_))
  {
    TOBAS_ERROR("Failed to get \"", kGPSwOnKey, "\".");
    return false;
  }
  if (!pt_.get(kGPSwOffKey, gpsw_off_))
  {
    TOBAS_ERROR("Failed to get \"", kGPSwOffKey, "\".");
    return false;
  }

  return true;
}

void RCInputHandlerNode::registerPubSub()
{
  rcin_pub_ = createPublisher<tobas_msgs::msg::RCInput>(tobas::kRcInputTopic);
  sbus_sub_ = createSubscriber(hal::kSBUSTopic, &self::sbusCb, this);
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
    sbus->data[real::kRcChannelThrot], throt_range_.lower, throt_range_.upper, tobas::kRCInputMin, tobas::kRCInputMax);
  rcin_msg->mode = tobas_std::closestIndex(modes_, sbus->data[real::kRcChannelMode]);
  rcin_msg->e_stop =
    abs(sbus->data[real::kRcChannelEStop] - estop_on_) < abs(sbus->data[real::kRcChannelEStop] - estop_off_);
  rcin_msg->gpsw = abs(sbus->data[real::kRcChannelGPSw] - gpsw_on_) < abs(sbus->data[real::kRcChannelGPSw] - gpsw_off_);

  // Publish message
  rcin_pub_->publish(move(rcin_msg));
}

void RCInputHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& req,
  const SetParams::Response::SharedPtr& res)
{
  // Update parameters
  roll_range_.lower = req->roll_left;
  roll_range_.upper = req->roll_right;
  pitch_range_.lower = req->pitch_down;
  pitch_range_.upper = req->pitch_up;
  yaw_range_.lower = req->yaw_right;
  yaw_range_.upper = req->yaw_left;
  throt_range_.lower = req->throttle_down;
  throt_range_.upper = req->throttle_up;
  modes_.at(tobas::flight_mode_t::PROGRAM_MODE) = req->mode_program;
  modes_.at(tobas::flight_mode_t::STABILIZE_MODE) = req->mode_stabilize;
  modes_.at(tobas::flight_mode_t::ACROBAT_MODE) = req->mode_acrobat;
  estop_on_ = req->estop_on;
  estop_off_ = req->estop_off;
  gpsw_on_ = req->gpsw_on;
  gpsw_off_ = req->gpsw_off;

  // Save parameters
  pt_.set(kRollLeftKey, req->roll_left);
  pt_.set(kRollRightKey, req->roll_right);
  pt_.set(kPitchDownKey, req->pitch_down);
  pt_.set(kPitchUpKey, req->pitch_up);
  pt_.set(kYawRightKey, req->yaw_right);
  pt_.set(kYawLeftKey, req->yaw_left);
  pt_.set(kThrotDownKey, req->throttle_down);
  pt_.set(kThrotUpKey, req->throttle_up);
  pt_.set(kModeProgramKey, req->mode_program);
  pt_.set(kModeStabilizeKey, req->mode_stabilize);
  pt_.set(kModeAcrobatKey, req->mode_acrobat);
  pt_.set(kEStopOnKey, req->estop_on);
  pt_.set(kEStopOffKey, req->estop_off);
  pt_.set(kGPSwOnKey, req->gpsw_on);
  pt_.set(kGPSwOffKey, req->gpsw_off);
  if (!pt_.save())
  {
    res->success = false;
    res->message = "Failed to save parameters.";
    return;
  }

  if (rcin_pub_ == nullptr)
    registerPubSub();

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(RCInputHandlerNode)
