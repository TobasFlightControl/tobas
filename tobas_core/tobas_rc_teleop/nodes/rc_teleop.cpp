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
#include <tobas_std_tools/check.hpp>

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

using namespace std::chrono_literals;

namespace tobas
{
namespace rc
{
class RCTeleopNode : public BaseNode
{
  static constexpr double kArmThrotThresh = 0.04;  // 帯域 [-1, 1] の 2%
  static constexpr auto kArmDuration = 1s;
  static constexpr auto kDisarmDuration = 1s;

  static constexpr double kArmCommandInfoPeriod = 2.;  // [s]
  static constexpr double kWarnPeriod = 1.;            // [s]

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

  // rosparams
  std::map<FlightMode, RcCommand> modes_;

  // Mutables
  FlightMode cur_mode_;
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
  void initializeControllers();
  void requestArmingRotors(bool arming);
  void updateWithIdleCommand(const tobas_msgs::RCInput& rcin);
  void resetCurrentController(const tobas_msgs::RCInput& rcin);
  bool isFlightModeApplicable(FlightMode mode);

  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void setpointCb(const tobas_msgs::OdometryStamped::ConstSharedPtr& setpoint);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};

RCTeleopNode::RCTeleopNode(const rclcpp::NodeOptions& options) : super(node::kRcTeleop, nodeOptions_DParam(options))
{
  TOBAS_CHECK(mode2str_.size() == magic_enum::enum_count<FlightMode>());

  getStaticRosParams();
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

  TOBAS_CHECK(enumFromText(getStringParam("acrobat_mode"), modes_.at(FlightMode::kAcrobat)));
  TOBAS_CHECK(enumFromText(getStringParam("stabilize_mode"), modes_.at(FlightMode::kStabilize)));
  TOBAS_CHECK(enumFromText(getStringParam("loiter_mode"), modes_.at(FlightMode::kLoiter)));
}

void RCTeleopNode::initializeControllers()
{
  // 各フライトモードに対応するコントローラを設定
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

  // 設定値が存在する場合はそれを初期目標値とする
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

  // 通信が切れた場合，切れる前のコマンドが残らないようにステージを最初に戻す．
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
      // アームされていれば次のステージに以降
      // プログラムモードから制御を奪う場合のために，アームコマンドの確認の前に現在のアーム状態の確認を行う．
      if (arming_->data) {
        stage_ = kWaitForEnable;
        break;
      }

      // アームコマンドが入力されている場合
      if (
        std::max(std::abs(rcin->roll), std::abs(rcin->pitch)) < kArmThrotThresh &&
        rcin->yaw < kRcInputMin + kArmThrotThresh && rcin->throttle < kRcInputMin + kArmThrotThresh) {
        // アームコマンドが一定時間維持されていれば一度アームをリクエスト
        if (rcin->header.stamp - t_arm_start_ > kArmDuration) {
          TOBAS_INFO("Arming rotors...");
          requestArmingRotors(true);
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        // アーム可能な場合のみ時刻を初期化せず継続
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

        TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Arm commanded.");
        break;
      }
      else {
        t_arm_start_ = rcin->header.stamp;
        break;
      }
    }

    case kWaitForEnable: {
      // ディスアームされていればステージをリセット
      if (!arming_->data) {
        t_arm_start_ = rcin->header.stamp;
        stage_ = kCheckPrerequisites;
        break;
      }

      // RC入力が有効ならば次のステージへ
      if (rcin->enable) {
        // 手動モードに移行した瞬間にディスアームされるのを防ぐため，Killスイッチがオンの時は移行しない．
        if (rcin->kill) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Turn off the kill switch before enabling RC control.");
          break;
        }

        // 飛行モードが適用可能であることを確認
        if (!isFlightModeApplicable(rcin->mode)) {
          break;
        }

        resetCurrentController(*rcin);
        cur_mode_ = rcin->mode;
        TOBAS_INFO("First flight mode is set to \"", mode2str_.at(rcin->mode), "\".");

        t_disarm_start_ = rcin->header.stamp;
        stage_ = kRunning;
      }

      break;
    }

    case kRunning: {
      // ディスアームされていればステージをリセット
      if (!arming_->data) {
        t_arm_start_ = rcin->header.stamp;
        stage_ = kCheckPrerequisites;
        break;
      }

      // Killスイッチがオンならば即ディスアーム
      if (rcin->kill) {
        TOBAS_WARN_THROTTLE(kWarnPeriod, "The kill switch has been activated. Forcing disarm.");
        requestArmingRotors(false);
        break;
      }

      // Enableスイッチがオフならば待機モードに戻る
      if (!rcin->enable) {
        TOBAS_INFO("RC control is disabled.");
        stage_ = kWaitForEnable;
        break;
      }

      // フライトモードの変更があった場合，適用可能な場合に限り変更する．
      // 適用できない場合は前のフライトモードを継続する．
      if (rcin->mode != cur_mode_ && isFlightModeApplicable(rcin->mode)) {
        resetCurrentController(*rcin);
        cur_mode_ = rcin->mode;
        TOBAS_INFO("Flight mode changed to \"", mode2str_.at(rcin->mode), "\".");
      }

      if (landed_->landed && rcin->throttle < kRcInputMin + kArmThrotThresh) {  // 地上でゼロスロットルの場合
        // 安全のためアイドルコマンドを送信
        updateWithIdleCommand(*rcin);

        // さらにディスアームコマンドの場合
        if (
          std::max(std::abs(rcin->roll), std::abs(rcin->pitch)) < kArmThrotThresh &&
          rcin->yaw > kRcInputMax - kArmThrotThresh) {
          TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Disarm commanded.");
          if (rcin->header.stamp - t_disarm_start_ > kDisarmDuration) {  // 一定時間維持されていればリクエスト
            TOBAS_INFO("Disarming rotors...");
            requestArmingRotors(false);
            t_disarm_start_ = rcin->header.stamp;
          }
          break;  // ディスアームコマンドの開始時刻を更新せずに抜ける
        }
      }
      else {  // それ以外は普通にコマンド送信
        controllers_.at(cur_mode_)->update(*rcin, odom_->odom.odom, landed_->landed);
      }

      // ディスアームコマンドの開始時刻を更新して抜ける
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
