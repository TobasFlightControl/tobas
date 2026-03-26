#include <magic_enum/magic_enum.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_constants/rc_input.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/range.hpp>

#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

using namespace tobas::real::handler::rcin;
namespace fs = std::filesystem;

namespace tobas
{
namespace real
{
class RCInputHandlerNode : public BaseNode
{
  using self = RCInputHandlerNode;
  using super = BaseNode;
  using SetParams = tobas_real_msgs::srv::SetRcInputParams;

public:
  explicit RCInputHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  st::Range<uint16_t> roll_;
  st::Range<uint16_t> pitch_;
  st::Range<uint16_t> yaw_;
  st::Range<uint16_t> throt_;
  std::map<tobas::FlightMode, uint16_t> modes_;
  uint16_t sub_mode_on_, sub_mode_off_;
  uint16_t enable_on_, enable_off_;
  uint16_t kill_on_, kill_off_;
  std::array<uint16_t, tobas::kMaxNumOfGpsw> gpsw_on_, gpsw_off_;

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::RCInput> rcin_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Sbus> sbus_sub_;
  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  bool getConfig();
  void registerPubSub();
  tobas::FlightMode getClosestFlightMode(uint16_t period);

  void sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus);
  void setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res);
};

RCInputHandlerNode::RCInputHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_rcin_handler", nodeOptions_Default(options))
{
  // Initialize property tree
  const auto cfg_dir = linux::isSuperUser() ? fs::path(tobas::kConfigDirRoot) : ros2::expandUser(tobas::kConfigDirHome);
  if (!pt_.initialize((cfg_dir / kConfigFileName))) {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
    return;
  }

  // Initialize mode map
  for (const auto mode : magic_enum::enum_values<tobas::FlightMode>()) {
    modes_[mode];
  }

  // Register service server
  set_params_ss_ = createService<SetParams>(kSetParamSrv, &self::setParamsCb, this);

  // Try to get configuration
  if (!getConfig()) {
    TOBAS_ERROR("Failed to get configuration. This node will not work until they are set.");
    return;
  }

  // Register publishers and subscribers if getting configuration is successful
  registerPubSub();
}

bool RCInputHandlerNode::getConfig()
{
  if (!pt_.get(ns(), kRollLeftKey, roll_.lower)) {
    TOBAS_ERROR("Failed to get \"", kRollLeftKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kRollRightKey, roll_.upper)) {
    TOBAS_ERROR("Failed to get \"", kRollRightKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kPitchUpKey, pitch_.upper)) {
    TOBAS_ERROR("Failed to get \"", kPitchUpKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kPitchDownKey, pitch_.lower)) {
    TOBAS_ERROR("Failed to get \"", kPitchDownKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kYawLeftKey, yaw_.upper)) {
    TOBAS_ERROR("Failed to get \"", kYawLeftKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kYawRightKey, yaw_.lower)) {
    TOBAS_ERROR("Failed to get \"", kYawRightKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kThrotUpKey, throt_.lower)) {
    TOBAS_ERROR("Failed to get \"", kThrotUpKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kThrotDownKey, throt_.upper)) {
    TOBAS_ERROR("Failed to get \"", kThrotDownKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kModeAcrobatKey, modes_.at(tobas::FlightMode::kAcrobat))) {
    TOBAS_ERROR("Failed to get \"", kModeAcrobatKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kModeStabilizeKey, modes_.at(tobas::FlightMode::kStabilize))) {
    TOBAS_ERROR("Failed to get \"", kModeStabilizeKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kModeLoiterKey, modes_.at(tobas::FlightMode::kLoiter))) {
    TOBAS_ERROR("Failed to get \"", kModeLoiterKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kSubModeOnKey, sub_mode_on_)) {
    TOBAS_ERROR("Failed to get \"", kSubModeOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kSubModeOffKey, sub_mode_off_)) {
    TOBAS_ERROR("Failed to get \"", kSubModeOffKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kEnableOnKey, enable_on_)) {
    TOBAS_ERROR("Failed to get \"", kEnableOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kEnableOffKey, enable_off_)) {
    TOBAS_ERROR("Failed to get \"", kEnableOffKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kKillOnKey, kill_on_)) {
    TOBAS_ERROR("Failed to get \"", kKillOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kKillOffKey, kill_off_)) {
    TOBAS_ERROR("Failed to get \"", kKillOffKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), kGpswOnKey, gpsw_on_)) {
    TOBAS_ERROR("Failed to get \"", kGpswOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), kGpswOffKey, gpsw_off_)) {
    TOBAS_ERROR("Failed to get \"", kGpswOffKey, "\".");
    return false;
  }

  return true;
}

void RCInputHandlerNode::registerPubSub()
{
  rcin_pub_ = createPublisher<tobas_msgs::RCInput>(topic::kRcInput);
  sbus_sub_ = createSubscriber(topic::kSbus, &self::sbusCb, this);
}

tobas::FlightMode RCInputHandlerNode::getClosestFlightMode(uint16_t period)
{
  tobas::FlightMode res = tobas::FlightMode::kLoiter;  // コンパイラ警告を抑制するために適当に初期化
  auto min_dist = std::numeric_limits<uint16_t>::max();

  for (const auto& [mode, period_ref] : modes_) {
    const auto dist = std::abs(period - period_ref);
    if (dist < min_dist) {
      min_dist = dist;
      res = mode;
    }
  }

  return res;
}

void RCInputHandlerNode::sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus)
{
  auto rcin_msg = std::make_unique<tobas_msgs::RCInput>();
  rcin_msg->header = sbus->header;

  if (sbus->frame_lost) {
    rcin_msg->ok = false;
  }
  else {
    rcin_msg->ok = true;

    const auto& roll = sbus->periods[tobas::kRcChannelRoll];
    const auto& pitch = sbus->periods[tobas::kRcChannelPitch];
    const auto& throt = sbus->periods[tobas::kRcChannelThrot];
    const auto& yaw = sbus->periods[tobas::kRcChannelYaw];
    const auto& mode = sbus->periods[tobas::kRcChannelMode];
    const auto& sub_mode = sbus->periods[tobas::kRcChannelSubMode];
    const auto& enable = sbus->periods[tobas::kRcChannelEnable];
    const auto& kill = sbus->periods[tobas::kRcChannelKill];

    rcin_msg->roll = math::remap<double>(roll, roll_.lower, roll_.upper, tobas::kRcInputMin, tobas::kRcInputMax);
    rcin_msg->pitch = math::remap<double>(pitch, pitch_.lower, pitch_.upper, tobas::kRcInputMin, tobas::kRcInputMax);
    rcin_msg->throttle = math::remap<double>(throt, throt_.lower, throt_.upper, tobas::kRcInputMin, tobas::kRcInputMax);
    rcin_msg->yaw = math::remap<double>(yaw, yaw_.lower, yaw_.upper, tobas::kRcInputMin, tobas::kRcInputMax);

    rcin_msg->mode = getClosestFlightMode(mode);
    rcin_msg->sub_mode = std::abs(sub_mode - sub_mode_on_) < std::abs(sub_mode - sub_mode_off_);
    rcin_msg->enable = std::abs(enable - enable_on_) < std::abs(enable - enable_off_);
    rcin_msg->kill = std::abs(kill - kill_on_) < std::abs(kill - kill_off_);

    for (size_t i = 0; i < tobas::kMaxNumOfGpsw; ++i) {
      const auto& gpsw = sbus->periods[tobas::kRcChannelGpsw + i];
      rcin_msg->gpsw[i] = std::abs(gpsw - gpsw_on_[i]) < std::abs(gpsw - gpsw_off_[i]);
    }
  }

  rcin_pub_->publish(std::move(rcin_msg));
}

void RCInputHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& req,
  const SetParams::Response::SharedPtr& res)
{
  // Update parameters
  roll_.lower = req->roll_left;
  roll_.upper = req->roll_right;
  pitch_.upper = req->pitch_up;
  pitch_.lower = req->pitch_down;
  yaw_.upper = req->yaw_left;
  yaw_.lower = req->yaw_right;
  throt_.lower = req->throttle_up;
  throt_.upper = req->throttle_down;
  modes_.at(tobas::FlightMode::kAcrobat) = req->mode_acrobat;
  modes_.at(tobas::FlightMode::kStabilize) = req->mode_stabilize;
  modes_.at(tobas::FlightMode::kLoiter) = req->mode_loiter;
  sub_mode_on_ = req->sub_mode_on;
  sub_mode_off_ = req->sub_mode_off;
  enable_on_ = req->enable_on;
  enable_off_ = req->enable_off;
  kill_on_ = req->kill_on;
  kill_off_ = req->kill_off;
  gpsw_on_ = req->gpsw_on;
  gpsw_off_ = req->gpsw_off;

  // Save parameters
  pt_.set(ns(), kRollLeftKey, req->roll_left);
  pt_.set(ns(), kRollRightKey, req->roll_right);
  pt_.set(ns(), kPitchUpKey, req->pitch_up);
  pt_.set(ns(), kPitchDownKey, req->pitch_down);
  pt_.set(ns(), kYawLeftKey, req->yaw_left);
  pt_.set(ns(), kYawRightKey, req->yaw_right);
  pt_.set(ns(), kThrotUpKey, req->throttle_up);
  pt_.set(ns(), kThrotDownKey, req->throttle_down);
  pt_.set(ns(), kModeAcrobatKey, req->mode_acrobat);
  pt_.set(ns(), kModeStabilizeKey, req->mode_stabilize);
  pt_.set(ns(), kModeLoiterKey, req->mode_loiter);
  pt_.set(ns(), kSubModeOnKey, req->sub_mode_on);
  pt_.set(ns(), kSubModeOffKey, req->sub_mode_off);
  pt_.set(ns(), kEnableOnKey, req->enable_on);
  pt_.set(ns(), kEnableOffKey, req->enable_off);
  pt_.set(ns(), kKillOnKey, req->kill_on);
  pt_.set(ns(), kKillOffKey, req->kill_off);
  pt_.set(ns(), kGpswOnKey, req->gpsw_on);
  pt_.set(ns(), kGpswOffKey, req->gpsw_off);
  if (!pt_.save()) {
    res->success = false;
    res->message = "Failed to save parameters.";
    return;
  }

  if (!rcin_pub_) {
    registerPubSub();
  }

  res->success = true;
  res->message.clear();
}
}  // namespace real
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::real::RCInputHandlerNode)
