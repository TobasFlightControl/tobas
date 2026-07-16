// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <eigen3/Eigen/Eigen>
#include <magic_enum/magic_enum.hpp>

#include <tobas_constants/node.hpp>
#include <tobas_constants/rc_command.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

#include "tobas_rc_teleop/accel_angle.hpp"
#include "tobas_rc_teleop/accel_pitch_yaw.hpp"
#include "tobas_rc_teleop/accel_rate.hpp"
#include "tobas_rc_teleop/accel_yaw.hpp"
#include "tobas_rc_teleop/angle_throttle.hpp"
#include "tobas_rc_teleop/angle_throttle_vector.hpp"
#include "tobas_rc_teleop/pos_vel_acc_angle.hpp"
#include "tobas_rc_teleop/pos_vel_acc_pitch_yaw.hpp"
#include "tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "tobas_rc_teleop/rate_throttle.hpp"
#include "tobas_rc_teleop/rate_throttle_vector.hpp"
#include "tobas_rc_teleop/speed_roll_dpitch.hpp"

namespace tobas
{
namespace rc
{
class RCTeleopNode : public BaseNode
{
  static constexpr double kArmCommandInfoPeriod = 2.0;  // [s]
  static constexpr double kWarnPeriod = 1.0;            // [s]

  using self = RCTeleopNode;
  using super = BaseNode;

public:
  explicit RCTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum Stage
  {
    // Pre-Arm
    kCheckPrerequisites,
    kWaitForArming,

    // Post-Arm
    kWaitForEnable,
    kRunning,
  } stage_ = kCheckPrerequisites;

  const std::map<FlightMode, const char*> mode2str_{
    { FlightMode::kAcrobat, "Acrobat" },
    { FlightMode::kStabilize, "Stabilize" },
    { FlightMode::kLoiter, "Loiter" },
  };

  // Static parameters
  std::map<FlightMode, RcCommand> modes_;
  double arm_duration_, disarm_duration_;  // [s]

  // Dynamic parameters
  double arm_throt_thresh_;
  double arm_throt_hyst_;  // Hysteresis to prevent chattering

  // Mutables
  FlightMode cur_mode_;
  bool is_zero_throt_ = true;
  builtin_interfaces::msg::Time t_arm_start_, t_disarm_start_;
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_;
  tobas_msgs::OdometryStamped::ConstSharedPtr setpoint_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::VehicleHealth::ConstSharedPtr health_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;

  // Controllers
  std::map<FlightMode, std::unique_ptr<BaseController>> controllers_;

  // PubSub
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryStamped> setpoint_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;

  // Service
  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  void getStaticRosParams();
  void defineDynamicRosParams();
  void initializeControllers();
  void requestArmingRotors(bool arming);
  void updateWithIdleCommand(const tobas_msgs::RCInput& rcin);
  void resetCurrentController(const tobas_msgs::RCInput& rcin);
  bool isFlightModeApplicable(FlightMode mode);

  bool armThrottleThresholdCb(const double& p);
  bool armThrottleHysteresisCb(const double& p);

  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void setpointCb(const tobas_msgs::OdometryStamped::ConstSharedPtr& setpoint);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};

RCTeleopNode::RCTeleopNode(const rclcpp::NodeOptions& options) : super(node::kRcTeleop, nodeOptions_DParam(options))
{
  TOBAS_ASSERT(mode2str_.size() == magic_enum::enum_count<FlightMode>());

  getStaticRosParams();
  defineDynamicRosParams();
  initializeControllers();

  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
  setpoint_sub_ = createSubscriber(topic::kTrajSetpoint, &self::setpointCb, this);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  health_sub_ = createSubscriber(topic::kVehicleHealth, &self::healthCb, this);
  rcin_sub_ = createSubscriber(topic::kRcInput, &self::rcInputCb, this);
  landed_sub_ = createSubscriber(topic::kLanded, &self::landedCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(service::kSetArm);
}

void RCTeleopNode::getStaticRosParams()
{
  for (const auto mode : magic_enum::enum_values<FlightMode>()) {
    modes_[mode];
  }

  TOBAS_ASSERT(enumFromText(getStringParam("acrobat_mode"), modes_.at(FlightMode::kAcrobat)));
  TOBAS_ASSERT(enumFromText(getStringParam("stabilize_mode"), modes_.at(FlightMode::kStabilize)));
  TOBAS_ASSERT(enumFromText(getStringParam("loiter_mode"), modes_.at(FlightMode::kLoiter)));

  arm_duration_ = getDoubleParam("arm_duration");
  disarm_duration_ = getDoubleParam("disarm_duration");
}

void RCTeleopNode::defineDynamicRosParams()
{
  // Make this configurable because some transmitters cannot detect the arm position if the threshold is too small.
  addDynamicDoubleParam("common/arm_throttle_threshold", &self::armThrottleThresholdCb, this, 0.5, 2, 1, 6, " %");
  addDynamicDoubleParam("common/arm_throttle_hysteresis", &self::armThrottleHysteresisCb, this, 0.5, 2, 1, 6, " %");
}

void RCTeleopNode::initializeControllers()
{
  // Set the controller for each flight mode.
  for (const auto& [mode, cmd] : modes_) {
    switch (cmd) {
      case RcCommand::kRateThrottle:
        controllers_[mode] = std::make_unique<RateThrottleController>();
        break;
      case RcCommand::kRateThrottleVector:
        controllers_[mode] = std::make_unique<RateThrottleVectorController>();
        break;
      case RcCommand::kAngleThrottle:
        controllers_[mode] = std::make_unique<AngleThrottleController>();
        break;
      case RcCommand::kAngleThrottleVector:
        controllers_[mode] = std::make_unique<AngleThrottleVectorController>();
        break;
      case RcCommand::kAccelYaw:
        controllers_[mode] = std::make_unique<AccelYawController>();
        break;
      case RcCommand::kAccelPitchYaw:
        controllers_[mode] = std::make_unique<AccelPitchYawController>();
        break;
      case RcCommand::kPosVelAccYaw:
        controllers_[mode] = std::make_unique<PosVelAccYawController>();
        break;
      case RcCommand::kPosVelAccPitchYaw:
        controllers_[mode] = std::make_unique<PosVelAccPitchYawController>();
        break;
      case RcCommand::kAccelRate:
        controllers_[mode] = std::make_unique<AccelRateController>();
        break;
      case RcCommand::kAccelAngle:
        controllers_[mode] = std::make_unique<AccelAngleController>();
        break;
      case RcCommand::kPosVelAccAngle:
        controllers_[mode] = std::make_unique<PosVelAccAngleController>();
        break;
      case RcCommand::kSpeedRollDPitch:
        controllers_[mode] = std::make_unique<SpeedRollDeltaPitchController>();
        break;
      default:
        TOBAS_EXIT("Invalid flight mode: ", (int)mode);
    }

    controllers_.at(mode)->initialize(this, mode);
  }
}

void RCTeleopNode::requestArmingRotors(bool arming)
{
  if (!set_arm_sc_->service_is_ready()) {
    TOBAS_ERROR("\"", service::kSetArm, "\" is not ready.");
    return;
  }

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  set_arm_sc_->async_send_request(req);
}

void RCTeleopNode::updateWithIdleCommand(const tobas_msgs::RCInput& rcin)
{
  auto idle_rcin = rcin;

  idle_rcin.roll = kRCInputMid;
  idle_rcin.pitch = kRCInputMid;
  idle_rcin.yaw = kRCInputMid;
  idle_rcin.throttle = kRcInputMin;

  controllers_.at(cur_mode_)->update(idle_rcin, odom_->odom.odom, landed_->landed);
}

void RCTeleopNode::resetCurrentController(const tobas_msgs::RCInput& rcin)
{
  auto init_setpoint = odom_->odom.odom;

  // Use the setpoint as the initial target value if it exists.
  if (setpoint_) {
    const auto& sp = setpoint_->odom;
    if (sp.frame.p.isFinite()) {
      init_setpoint.frame.p = sp.frame.p;
    }
    if (sp.frame.M.isFinite()) {
      init_setpoint.frame.M = sp.frame.M;
    }
    if (sp.twist.vel.isFinite()) {
      init_setpoint.twist.vel = sp.twist.vel;
    }
    if (sp.twist.rot.isFinite()) {
      init_setpoint.twist.rot = sp.twist.rot;
    }
    if (sp.accel.linear.isFinite()) {
      init_setpoint.accel.linear = sp.accel.linear;
    }
    if (sp.accel.angular.isFinite()) {
      init_setpoint.accel.angular = sp.accel.angular;
    }
  }

  controllers_.at(rcin.mode)->reset(rcin.header.stamp, init_setpoint, landed_->landed);
}

bool RCTeleopNode::isFlightModeApplicable(FlightMode mode)
{
  const auto& controller = controllers_.at(mode);

  if (controller->requirePosition()) {
    switch (health_->position_accuracy) {
      case tobas_msgs::msg::VehicleHealth::PASSED:
        break;
      case tobas_msgs::msg::VehicleHealth::FAILED:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because the position estimation is inaccurate.");
        return false;
      case tobas_msgs::msg::VehicleHealth::IGNORED:
      case tobas_msgs::msg::VehicleHealth::UNKNOWN:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod,
          mode2str_.at(mode),
          " mode cannot be applied because the position estimation accuracy has not been checked.");
        return false;
      default:
        TOBAS_ERROR("The position accuracy state is invalid: ", (int)health_->position_accuracy);
        return false;
    }
  }

  if (controller->requireVelocity()) {
    switch (health_->velocity_accuracy) {
      case tobas_msgs::msg::VehicleHealth::PASSED:
        break;
      case tobas_msgs::msg::VehicleHealth::FAILED:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because the velocity estimation is inaccurate.");
        return false;
      case tobas_msgs::msg::VehicleHealth::IGNORED:
      case tobas_msgs::msg::VehicleHealth::UNKNOWN:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod,
          mode2str_.at(mode),
          " mode cannot be applied because the velocity estimation accuracy has not been checked.");
        return false;
      default:
        TOBAS_ERROR("The velocity accuracy state is invalid: ", (int)health_->position_accuracy);
        return false;
    }
  }

  if (controller->requireAttitude()) {
    switch (health_->attitude_accuracy) {
      case tobas_msgs::msg::VehicleHealth::PASSED:
        break;
      case tobas_msgs::msg::VehicleHealth::FAILED:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because the attitude estimation is inaccurate.");
        return false;
      case tobas_msgs::msg::VehicleHealth::IGNORED:
      case tobas_msgs::msg::VehicleHealth::UNKNOWN:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod,
          mode2str_.at(mode),
          " mode cannot be applied because the attitude estimation accuracy has not been checked.");
        return false;
      default:
        TOBAS_ERROR("The attitude accuracy state is invalid: ", (int)health_->position_accuracy);
        return false;
    }
  }

  if (controller->requireHeading()) {
    switch (health_->heading_accuracy) {
      case tobas_msgs::msg::VehicleHealth::PASSED:
        break;
      case tobas_msgs::msg::VehicleHealth::FAILED:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because the heading estimation is inaccurate.");
        return false;
      case tobas_msgs::msg::VehicleHealth::IGNORED:
      case tobas_msgs::msg::VehicleHealth::UNKNOWN:
        TOBAS_WARN_THROTTLE(
          kWarnPeriod,
          mode2str_.at(mode),
          " mode cannot be applied because the heading estimation accuracy has not been checked.");
        return false;
      default:
        TOBAS_ERROR("The heading accuracy state is invalid: ", (int)health_->position_accuracy);
        return false;
    }
  }

  return true;
}

bool RCTeleopNode::armThrottleThresholdCb(const double& p)
{
  arm_throt_thresh_ = kRcInputRange * (p / 100.0);
  return true;
}

bool RCTeleopNode::armThrottleHysteresisCb(const double& p)
{
  arm_throt_hyst_ = kRcInputRange * (p / 100.0);
  return true;
}

void RCTeleopNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void RCTeleopNode::setpointCb(const tobas_msgs::OdometryStamped::ConstSharedPtr& setpoint)
{
  setpoint_ = setpoint;
}

void RCTeleopNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (!arming->data) {
    setpoint_.reset();
  }
}

void RCTeleopNode::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  health_ = health;

  // If the link is lost, reset the stage so commands from before the loss do not remain active.
  if (health->radio_link == tobas_msgs::msg::VehicleHealth::FAILED) {
    if (stage_ != kCheckPrerequisites) {
      stage_ = kCheckPrerequisites;
      TOBAS_WARN("The radio control command has been reset because the link was lost.");
    }
  }
}

void RCTeleopNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

void RCTeleopNode::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  if (!rcin->ok) {
    return;
  }

  switch (stage_) {
    case kCheckPrerequisites: {
      if (!odom_) {
        TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Waiting for odometry.");
        break;
      }
      if (!arming_) {
        TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Waiting for arming status.");
        break;
      }
      if (!health_) {
        TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Warting for vehicle health status.");
        break;
      }
      if (!landed_) {
        TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Warting for landing status.");
        break;
      }

      t_arm_start_ = rcin->header.stamp;
      stage_ = kWaitForArming;
      break;
    }

    case kWaitForArming: {
      // If already armed, move to the next stage.
      // Check the current arming state before checking the arm command so manual control can take over from program mode.
      if (arming_->data) {
        stage_ = kWaitForEnable;
        break;
      }

      // If the arm command is being input
      if (
        std::max(std::abs(rcin->roll), std::abs(rcin->pitch)) < arm_throt_thresh_ &&
        rcin->yaw < kRcInputMin + arm_throt_thresh_ && rcin->throttle < kRcInputMin + arm_throt_thresh_) {
        // Request arming once if the arm command has been held for a fixed time.
        if ((rcin->header.stamp - t_arm_start_).seconds() > arm_duration_) {
          TOBAS_INFO("Arming rotors...");
          requestArmingRotors(true);
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        // Continue without resetting the time only when arming is possible.
        if (!rcin->enable) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Please turn on the enable switch before arming.");
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        if (rcin->kill) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Please turn off the kill switch before arming.");
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        if (!health_->ok) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Cannot arm because pre-arm check failed.");
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        if (!isFlightModeApplicable(rcin->mode)) {
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Arm stick position detected. Hold to arm.");
        break;
      }
      else {
        t_arm_start_ = rcin->header.stamp;
        break;
      }
    }

    case kWaitForEnable: {
      // Reset the stage if disarmed.
      if (!arming_->data) {
        t_arm_start_ = rcin->header.stamp;
        stage_ = kCheckPrerequisites;
        break;
      }

      // Move to the next stage if RC input is enabled.
      if (rcin->enable) {
        // Do not transition while the kill switch is on, to avoid disarming immediately after entering manual mode.
        if (rcin->kill) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Turn off the kill switch before enabling RC control.");
          break;
        }

        // Check that the flight mode is applicable.
        if (!isFlightModeApplicable(rcin->mode)) {
          break;
        }

        resetCurrentController(*rcin);
        cur_mode_ = rcin->mode;
        TOBAS_INFO("First flight mode has been set to \"", mode2str_.at(rcin->mode), "\".");

        t_disarm_start_ = rcin->header.stamp;
        stage_ = kRunning;
      }

      break;
    }

    case kRunning: {
      // Reset the stage if disarmed.
      if (!arming_->data) {
        t_arm_start_ = rcin->header.stamp;
        stage_ = kCheckPrerequisites;
        break;
      }

      // Immediately disarm if the kill switch is on.
      if (rcin->kill) {
        TOBAS_WARN_THROTTLE(kWarnPeriod, "The kill switch has been activated. Forcing disarm.");
        requestArmingRotors(false);
        break;
      }

      // Return to standby mode if the enable switch is off.
      if (!rcin->enable) {
        TOBAS_INFO("RC control is disabled.");
        stage_ = kWaitForEnable;
        break;
      }

      // If the flight mode changes, apply the change only when the new mode is applicable.
      // Otherwise, keep the previous flight mode.
      if (rcin->mode != cur_mode_ && isFlightModeApplicable(rcin->mode)) {
        resetCurrentController(*rcin);
        cur_mode_ = rcin->mode;
        TOBAS_INFO("Flight mode changed to \"", mode2str_.at(rcin->mode), "\".");
      }

      const auto zero_throt_thresh =
        kRcInputMin + arm_throt_thresh_ + static_cast<int>(is_zero_throt_) * arm_throt_hyst_;
      if (landed_->landed && rcin->throttle < zero_throt_thresh) {  // Zero throttle on the ground
        is_zero_throt_ = true;

        // Send an idle command for safety.
        updateWithIdleCommand(*rcin);

        // If this is also the disarm command
        if (
          std::max(std::abs(rcin->roll), std::abs(rcin->pitch)) < arm_throt_thresh_ &&
          rcin->yaw > kRcInputMax - arm_throt_thresh_) {
          TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Disarm stick position detected. Hold to disarm.");
          if ((rcin->header.stamp - t_disarm_start_).seconds() > disarm_duration_) {  // Request if held long enough.
            TOBAS_INFO("Disarming rotors...");
            requestArmingRotors(false);
            t_disarm_start_ = rcin->header.stamp;
          }
          break;  // Exit without updating the disarm command start time.
        }
      }
      else {  // Otherwise, send the command normally.
        is_zero_throt_ = false;
        controllers_.at(cur_mode_)->update(*rcin, odom_->odom.odom, landed_->landed);
      }

      // Update the disarm command start time before exiting.
      t_disarm_start_ = rcin->header.stamp;
      break;
    }

    default: {
      TOBAS_ERROR("Invalid stage: ", (int)stage_);
      break;
    }
  }
}
}  // namespace rc
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::rc::RCTeleopNode)
