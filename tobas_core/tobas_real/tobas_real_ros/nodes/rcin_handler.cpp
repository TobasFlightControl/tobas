// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <magic_enum.hpp>

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
  std::map<FlightMode, uint16_t> modes_;
  uint16_t sub_mode_on_, sub_mode_off_;
  uint16_t enable_on_, enable_off_;
  uint16_t kill_on_, kill_off_;
  std::array<uint16_t, kMaxNumOfGpsw> gpsw_on_, gpsw_off_;

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::RCInput> rcin_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Sbus> sbus_sub_;
  ros2::ServiceServerPtr<SetParams> set_params_ss_;

  bool getConfig();
  void registerPubSub();
  FlightMode getClosestFlightMode(uint16_t period);

  void sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus);
  void setParamsCb(const SetParams::Request::ConstSharedPtr& req, const SetParams::Response::SharedPtr& res);
};

RCInputHandlerNode::RCInputHandlerNode(const rclcpp::NodeOptions& options)
  : super("real_rcin_handler", nodeOptions_Default(options))
{
  // Initialize property tree.
  const auto cfg_dir = linux::isSuperUser() ? fs::path(kConfigDirRoot) : ros2::expandUser(kConfigDirHome);
  if (!pt_.initialize((cfg_dir / handler::rcin::kConfigFileName))) {
    TOBAS_ERROR("Failed to initialize property tree. This node will not work.");
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
  if (!pt_.get(ns(), handler::rcin::kRollLeftKey, roll_.lower)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kRollLeftKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kRollRightKey, roll_.upper)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kRollRightKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kPitchUpKey, pitch_.upper)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kPitchUpKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kPitchDownKey, pitch_.lower)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kPitchDownKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kYawLeftKey, yaw_.upper)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kYawLeftKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kYawRightKey, yaw_.lower)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kYawRightKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kThrotUpKey, throt_.lower)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kThrotUpKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kThrotDownKey, throt_.upper)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kThrotDownKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kModeAcrobatKey, modes_.at(FlightMode::kAcrobat))) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kModeAcrobatKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kModeStabilizeKey, modes_.at(FlightMode::kStabilize))) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kModeStabilizeKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kModeLoiterKey, modes_.at(FlightMode::kLoiter))) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kModeLoiterKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kSubModeOnKey, sub_mode_on_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kSubModeOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kSubModeOffKey, sub_mode_off_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kSubModeOffKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kEnableOnKey, enable_on_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kEnableOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kEnableOffKey, enable_off_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kEnableOffKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kKillOnKey, kill_on_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kKillOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kKillOffKey, kill_off_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kKillOffKey, "\".");
    return false;
  }

  if (!pt_.get(ns(), handler::rcin::kGpswOnKey, gpsw_on_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kGpswOnKey, "\".");
    return false;
  }
  if (!pt_.get(ns(), handler::rcin::kGpswOffKey, gpsw_off_)) {
    TOBAS_ERROR("Failed to get \"", handler::rcin::kGpswOffKey, "\".");
    return false;
  }

  return true;
}

void RCInputHandlerNode::registerPubSub()
{
  rcin_pub_ = createPublisher<tobas_msgs::RCInput>(topic::kRcInput);
  sbus_sub_ = createSubscriber(topic::kSbus, &self::sbusCb, this);
}

FlightMode RCInputHandlerNode::getClosestFlightMode(uint16_t period)
{
  FlightMode res = FlightMode::kLoiter;  // Initialize to suppress compiler warnings.
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

    const auto& roll = sbus->periods[kRcChannelRoll];
    const auto& pitch = sbus->periods[kRcChannelPitch];
    const auto& throt = sbus->periods[kRcChannelThrot];
    const auto& yaw = sbus->periods[kRcChannelYaw];
    const auto& mode = sbus->periods[kRcChannelMode];
    const auto& sub_mode = sbus->periods[kRcChannelSubMode];
    const auto& enable = sbus->periods[kRcChannelEnable];
    const auto& kill = sbus->periods[kRcChannelKill];

    rcin_msg->roll = math::remap<double>(roll, roll_.lower, roll_.upper, kRcInputMin, kRcInputMax);
    rcin_msg->pitch = math::remap<double>(pitch, pitch_.lower, pitch_.upper, kRcInputMin, kRcInputMax);
    rcin_msg->throttle = math::remap<double>(throt, throt_.lower, throt_.upper, kRcInputMin, kRcInputMax);
    rcin_msg->yaw = math::remap<double>(yaw, yaw_.lower, yaw_.upper, kRcInputMin, kRcInputMax);

    rcin_msg->mode = getClosestFlightMode(mode);
    rcin_msg->sub_mode = std::abs(sub_mode - sub_mode_on_) < std::abs(sub_mode - sub_mode_off_);
    rcin_msg->enable = std::abs(enable - enable_on_) < std::abs(enable - enable_off_);
    rcin_msg->kill = std::abs(kill - kill_on_) < std::abs(kill - kill_off_);

    for (size_t i = 0; i < kMaxNumOfGpsw; ++i) {
      const auto& gpsw = sbus->periods[kRcChannelGpsw + i];
      rcin_msg->gpsw[i] = std::abs(gpsw - gpsw_on_[i]) < std::abs(gpsw - gpsw_off_[i]);
    }
  }

  rcin_pub_->publish(std::move(rcin_msg));
}

void RCInputHandlerNode::setParamsCb(
  const SetParams::Request::ConstSharedPtr& req,
  const SetParams::Response::SharedPtr& res)
{
  // Update parameters.
  roll_.lower = req->roll_left;
  roll_.upper = req->roll_right;
  pitch_.upper = req->pitch_up;
  pitch_.lower = req->pitch_down;
  yaw_.upper = req->yaw_left;
  yaw_.lower = req->yaw_right;
  throt_.lower = req->throttle_up;
  throt_.upper = req->throttle_down;
  modes_.at(FlightMode::kAcrobat) = req->mode_acrobat;
  modes_.at(FlightMode::kStabilize) = req->mode_stabilize;
  modes_.at(FlightMode::kLoiter) = req->mode_loiter;
  sub_mode_on_ = req->sub_mode_on;
  sub_mode_off_ = req->sub_mode_off;
  enable_on_ = req->enable_on;
  enable_off_ = req->enable_off;
  kill_on_ = req->kill_on;
  kill_off_ = req->kill_off;
  gpsw_on_ = req->gpsw_on;
  gpsw_off_ = req->gpsw_off;

  // Save parameters.
  pt_.set(ns(), handler::rcin::kRollLeftKey, req->roll_left);
  pt_.set(ns(), handler::rcin::kRollRightKey, req->roll_right);
  pt_.set(ns(), handler::rcin::kPitchUpKey, req->pitch_up);
  pt_.set(ns(), handler::rcin::kPitchDownKey, req->pitch_down);
  pt_.set(ns(), handler::rcin::kYawLeftKey, req->yaw_left);
  pt_.set(ns(), handler::rcin::kYawRightKey, req->yaw_right);
  pt_.set(ns(), handler::rcin::kThrotUpKey, req->throttle_up);
  pt_.set(ns(), handler::rcin::kThrotDownKey, req->throttle_down);
  pt_.set(ns(), handler::rcin::kModeAcrobatKey, req->mode_acrobat);
  pt_.set(ns(), handler::rcin::kModeStabilizeKey, req->mode_stabilize);
  pt_.set(ns(), handler::rcin::kModeLoiterKey, req->mode_loiter);
  pt_.set(ns(), handler::rcin::kSubModeOnKey, req->sub_mode_on);
  pt_.set(ns(), handler::rcin::kSubModeOffKey, req->sub_mode_off);
  pt_.set(ns(), handler::rcin::kEnableOnKey, req->enable_on);
  pt_.set(ns(), handler::rcin::kEnableOffKey, req->enable_off);
  pt_.set(ns(), handler::rcin::kKillOnKey, req->kill_on);
  pt_.set(ns(), handler::rcin::kKillOffKey, req->kill_off);
  pt_.set(ns(), handler::rcin::kGpswOnKey, req->gpsw_on);
  pt_.set(ns(), handler::rcin::kGpswOffKey, req->gpsw_off);
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
