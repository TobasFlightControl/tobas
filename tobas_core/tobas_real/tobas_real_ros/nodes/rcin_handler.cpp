// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <magic_enum/magic_enum.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_constants/rc_input.hpp>
#include <tobas_constants/time.hpp>
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

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace tobas::real
{
class RCInputHandlerNode : public BaseNode
{
  using self = RCInputHandlerNode;
  using super = BaseNode;
  using SetParams = tobas_real_msgs::srv::SetRcInputParams;

public:
  explicit RCInputHandlerNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  std::string section_;

  // Config
  st::Range<uint16_t> roll_;
  st::Range<uint16_t> pitch_;
  st::Range<uint16_t> yaw_;
  st::Range<uint16_t> throt_;
  std::map<FlightMode, uint16_t> modes_;
  uint16_t sub_mode_on_, sub_mode_off_;
  uint16_t enable_on_, enable_off_;
  uint16_t kill_on_, kill_off_;
  std::array<uint16_t, kMaxNumOfGpsw> gpsw_on_, gpsw_off_;

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::RCInput> rcin_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Sbus> sbus_sub_;
  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  ros2::TimerPtr timeout_timer_;

  bool getConfig();
  void registerPubSub();
  FlightMode getClosestFlightMode(uint16_t _period);

  void sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& _sbus);
  void timeoutTimerCb();
  void setParamsCb(const SetParams::Request::ConstSharedPtr& _req, const SetParams::Response::SharedPtr& _res);
};

RCInputHandlerNode::RCInputHandlerNode(const rclcpp::NodeOptions& _options)
  : super("real_rcin_handler", nodeOptions_Default(_options))
{
  section_ = getStringParam("section");

  // Initialize property tree.
  const auto cfg_dir = linux::isSuperUser() ? fs::path(kConfigDirRoot) : ros2::expandUser(kConfigDirHome);
  if (const auto result = pt_.initialize((cfg_dir / handler::rcin::kConfigFileName)); !result) {
    TOBAS_ERROR("Failed to initialize property tree: ", result.error(), ". This node will not work.");
    return;
  }

  // Initialize mode map.
  for (const auto mode : magic_enum::enum_values<FlightMode>()) {
    modes_[mode];
  }

  // Register service server.
  set_params_ss_ = createService<SetParams>(handler::rcin::kSetParamSrv, &self::setParamsCb, this);

  // Try to get configuration.
  if (!getConfig()) {
    TOBAS_ERROR("Failed to get configuration. This node will not work until they are set.");
    return;
  }

  // Register publishers and subscribers if getting configuration is successful.
  registerPubSub();
}

bool RCInputHandlerNode::getConfig()
{
  if (!pt_.get(section_, handler::rcin::kRollLeftKey, roll_.lower)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kRollLeftKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kRollRightKey, roll_.upper)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kRollRightKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kPitchUpKey, pitch_.upper)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kPitchUpKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kPitchDownKey, pitch_.lower)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kPitchDownKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kYawLeftKey, yaw_.upper)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kYawLeftKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kYawRightKey, yaw_.lower)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kYawRightKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kThrotUpKey, throt_.lower)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kThrotUpKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kThrotDownKey, throt_.upper)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kThrotDownKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kModeAcrobatKey, modes_.at(FlightMode::kAcrobat))) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kModeAcrobatKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kModeStabilizeKey, modes_.at(FlightMode::kStabilize))) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kModeStabilizeKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kModeLoiterKey, modes_.at(FlightMode::kLoiter))) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kModeLoiterKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kSubModeOnKey, sub_mode_on_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kSubModeOnKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kSubModeOffKey, sub_mode_off_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kSubModeOffKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kEnableOnKey, enable_on_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kEnableOnKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kEnableOffKey, enable_off_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kEnableOffKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kKillOnKey, kill_on_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kKillOnKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kKillOffKey, kill_off_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kKillOffKey, "'.");
    return false;
  }

  if (!pt_.get(section_, handler::rcin::kGpswOnKey, gpsw_on_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kGpswOnKey, "'.");
    return false;
  }
  if (!pt_.get(section_, handler::rcin::kGpswOffKey, gpsw_off_)) {
    TOBAS_ERROR("Failed to get '", handler::rcin::kGpswOffKey, "'.");
    return false;
  }

  return true;
}

void RCInputHandlerNode::registerPubSub()
{
  rcin_pub_ = createPublisher<tobas_msgs::RCInput>(topic::kRcInput);
  sbus_sub_ = createSubscriber(topic::kSbus, &self::sbusCb, this);
}

FlightMode RCInputHandlerNode::getClosestFlightMode(uint16_t _period)
{
  FlightMode res = FlightMode::kLoiter;  // Initialize to suppress compiler warnings.
  auto min_dist = std::numeric_limits<uint16_t>::max();

  for (const auto& [mode, period_ref] : modes_) {
    const auto dist = std::abs(_period - period_ref);
    if (dist < min_dist) {
      min_dist = dist;
      res = mode;
    }
  }

  return res;
}

void RCInputHandlerNode::sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& _sbus)
{
  if (!timeout_timer_) {
    // The longest S.BUS period is 14 ms.
    // Treat the receiver as abnormal if 3 or more frames are missing.
    // Always publish some message because subscribers are harder to implement as event-driven logic if messages may stop.
    timeout_timer_ = createWallTimer(14ms * 3, &self::timeoutTimerCb, this);
  }

  timeout_timer_->reset();

  auto rcin_msg = std::make_unique<tobas_msgs::RCInput>();
  rcin_msg->header = _sbus->header;

  if (_sbus->frame_lost) {
    rcin_msg->status = tobas_msgs::msg::RCInput::STATUS_FRAME_LOST;
  }
  else {
    rcin_msg->status = tobas_msgs::msg::RCInput::STATUS_OK;

    const auto& roll = _sbus->periods[kRcChannelRoll];
    const auto& pitch = _sbus->periods[kRcChannelPitch];
    const auto& throt = _sbus->periods[kRcChannelThrot];
    const auto& yaw = _sbus->periods[kRcChannelYaw];
    const auto& mode = _sbus->periods[kRcChannelMode];
    const auto& sub_mode = _sbus->periods[kRcChannelSubMode];
    const auto& enable = _sbus->periods[kRcChannelEnable];
    const auto& kill = _sbus->periods[kRcChannelKill];

    rcin_msg->roll = math::remap<double>(roll, roll_.lower, roll_.upper, kRcInputMin, kRcInputMax);
    rcin_msg->pitch = math::remap<double>(pitch, pitch_.lower, pitch_.upper, kRcInputMin, kRcInputMax);
    rcin_msg->throttle = math::remap<double>(throt, throt_.lower, throt_.upper, kRcInputMin, kRcInputMax);
    rcin_msg->yaw = math::remap<double>(yaw, yaw_.lower, yaw_.upper, kRcInputMin, kRcInputMax);

    rcin_msg->mode = getClosestFlightMode(mode);
    rcin_msg->sub_mode = std::abs(sub_mode - sub_mode_on_) < std::abs(sub_mode - sub_mode_off_);
    rcin_msg->enable = std::abs(enable - enable_on_) < std::abs(enable - enable_off_);
    rcin_msg->kill = std::abs(kill - kill_on_) < std::abs(kill - kill_off_);

    for (size_t i = 0; i < kMaxNumOfGpsw; ++i) {
      const auto& gpsw = _sbus->periods[kRcChannelGpsw + i];
      rcin_msg->gpsw[i] = std::abs(gpsw - gpsw_on_[i]) < std::abs(gpsw - gpsw_off_[i]);
    }
  }

  rcin_pub_->publish(std::move(rcin_msg));
}

void RCInputHandlerNode::timeoutTimerCb()
{
  TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Timed out waiting for S.BUS frames.");

  auto rcin_msg = std::make_unique<tobas_msgs::RCInput>();
  rcin_msg->header.stamp = now();
  rcin_msg->status = tobas_msgs::msg::RCInput::STATUS_TIMEOUT;
  rcin_pub_->publish(std::move(rcin_msg));
}

void RCInputHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& _req,
  const SetParams::Response::SharedPtr& _res)
{
  // Update parameters.
  roll_.lower = _req->roll_left;
  roll_.upper = _req->roll_right;
  pitch_.upper = _req->pitch_up;
  pitch_.lower = _req->pitch_down;
  yaw_.upper = _req->yaw_left;
  yaw_.lower = _req->yaw_right;
  throt_.lower = _req->throttle_up;
  throt_.upper = _req->throttle_down;
  modes_.at(FlightMode::kAcrobat) = _req->mode_acrobat;
  modes_.at(FlightMode::kStabilize) = _req->mode_stabilize;
  modes_.at(FlightMode::kLoiter) = _req->mode_loiter;
  sub_mode_on_ = _req->sub_mode_on;
  sub_mode_off_ = _req->sub_mode_off;
  enable_on_ = _req->enable_on;
  enable_off_ = _req->enable_off;
  kill_on_ = _req->kill_on;
  kill_off_ = _req->kill_off;
  gpsw_on_ = _req->gpsw_on;
  gpsw_off_ = _req->gpsw_off;

  // Save parameters.
  pt_.set(section_, handler::rcin::kRollLeftKey, _req->roll_left);
  pt_.set(section_, handler::rcin::kRollRightKey, _req->roll_right);
  pt_.set(section_, handler::rcin::kPitchUpKey, _req->pitch_up);
  pt_.set(section_, handler::rcin::kPitchDownKey, _req->pitch_down);
  pt_.set(section_, handler::rcin::kYawLeftKey, _req->yaw_left);
  pt_.set(section_, handler::rcin::kYawRightKey, _req->yaw_right);
  pt_.set(section_, handler::rcin::kThrotUpKey, _req->throttle_up);
  pt_.set(section_, handler::rcin::kThrotDownKey, _req->throttle_down);
  pt_.set(section_, handler::rcin::kModeAcrobatKey, _req->mode_acrobat);
  pt_.set(section_, handler::rcin::kModeStabilizeKey, _req->mode_stabilize);
  pt_.set(section_, handler::rcin::kModeLoiterKey, _req->mode_loiter);
  pt_.set(section_, handler::rcin::kSubModeOnKey, _req->sub_mode_on);
  pt_.set(section_, handler::rcin::kSubModeOffKey, _req->sub_mode_off);
  pt_.set(section_, handler::rcin::kEnableOnKey, _req->enable_on);
  pt_.set(section_, handler::rcin::kEnableOffKey, _req->enable_off);
  pt_.set(section_, handler::rcin::kKillOnKey, _req->kill_on);
  pt_.set(section_, handler::rcin::kKillOffKey, _req->kill_off);
  pt_.set(section_, handler::rcin::kGpswOnKey, _req->gpsw_on);
  pt_.set(section_, handler::rcin::kGpswOffKey, _req->gpsw_off);
  if (const auto result = pt_.save(); !result) {
    _res->success = false;
    _res->message = "Failed to save parameters: " + result.error();
    return;
  }

  if (!rcin_pub_) {
    registerPubSub();
  }

  _res->success = true;
  _res->message.clear();
}
}  // namespace tobas::real

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::real::RCInputHandlerNode)
