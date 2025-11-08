#include <eigen3/Eigen/Eigen>
#include <magic_enum/magic_enum.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_constants/rc_command.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/check.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

#include "tobas_rc_teleop/accel_angle.hpp"
#include "tobas_rc_teleop/accel_pitch_yaw.hpp"
#include "tobas_rc_teleop/accel_rate.hpp"
#include "tobas_rc_teleop/accel_yaw.hpp"
#include "tobas_rc_teleop/angle_throttle.hpp"
#include "tobas_rc_teleop/pos_vel_angle.hpp"
#include "tobas_rc_teleop/pos_vel_pitch_yaw.hpp"
#include "tobas_rc_teleop/pos_vel_yaw.hpp"
#include "tobas_rc_teleop/rate_throttle.hpp"
#include "tobas_rc_teleop/speed_roll_dpitch.hpp"

using namespace std::chrono_literals;

namespace tobas_rc_teleop
{
class RCTeleopNode : public tobas::BaseNode
{
  static constexpr double kArmThrotThresh = 0.1;  // [-]
  static constexpr double kArmDuration = 1.;      // [s]
  static constexpr double kDisarmDuration = 1.;   // [s]

  static constexpr double kPosStddevThresh = 3.;           // [m]
  static constexpr double kRotStddevThresh = M_PI / 12;    // [rad]
  static constexpr double kLinVelStddevThresh = 1.;        // [m/s]
  static constexpr double kAngVelStddevThresh = M_PI / 3;  // [rad/s]

  static constexpr double kArmCommandInfoPeriod = 2.;  // [s]
  static constexpr double kWarnPeriod = 1.;            // [s]

  static constexpr auto kRadioLinkLostTimeout = 1s;

  using self = RCTeleopNode;
  using super = tobas::BaseNode;

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

  const std::map<tobas::FlightMode, const char*> mode2str_{
    { tobas::FlightMode::kAcrobat, "Acrobat" },
    { tobas::FlightMode::kStabilize, "Stabilize" },
    { tobas::FlightMode::kLoiter, "Loiter" },
  };

  // rosparams
  std::map<tobas::FlightMode, tobas::RcCommand> modes_;

  // Mutables
  tobas::FlightMode cur_mode_;
  builtin_interfaces::msg::Time t_arm_start_, t_disarm_start_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::VehicleHealth::ConstSharedPtr health_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;

  // Controllers
  std::map<tobas::FlightMode, std::unique_ptr<BaseController>> controllers_;

  // PubSub
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;

  // Service
  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  // Timer
  ros2::TimerPtr rcin_timeout_timer_;

  void getStaticRosParams();
  void initializeControllers();
  void requestArmingRotors(bool arming);
  void updateWithIdleCommand(const tobas_msgs::RCInput& rcin);

  bool isFlightModeApplicable(tobas::FlightMode mode);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);

  void rcInputTimeoutTimerCb();
};

RCTeleopNode::RCTeleopNode(const rclcpp::NodeOptions& options) : super(tobas::node::kRcTeleop, options)
{
  TOBAS_CHECK(mode2str_.size() == magic_enum::enum_count<tobas::FlightMode>());

  getStaticRosParams();
  initializeControllers();

  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  health_sub_ = createSubscriber(tobas::kVehicleHealthTopic, &self::healthCb, this);
  rcin_sub_ = createSubscriber(tobas::kRcInputTopic, &self::rcInputCb, this);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);

  rcin_timeout_timer_ = createTimer(kRadioLinkLostTimeout, &self::rcInputTimeoutTimerCb, this);
}

void RCTeleopNode::getStaticRosParams()
{
  for (const auto mode : magic_enum::enum_values<tobas::FlightMode>()) {
    modes_[mode];
  }

  TOBAS_CHECK(tobas::enumFromText(getStringParam("acrobat_mode"), modes_.at(tobas::FlightMode::kAcrobat)));
  TOBAS_CHECK(tobas::enumFromText(getStringParam("stabilize_mode"), modes_.at(tobas::FlightMode::kStabilize)));
  TOBAS_CHECK(tobas::enumFromText(getStringParam("loiter_mode"), modes_.at(tobas::FlightMode::kLoiter)));
}

void RCTeleopNode::initializeControllers()
{
  // 各フライトモードに対応するコントローラを設定
  for (const auto& [mode, cmd] : modes_) {
    switch (cmd) {
      case tobas::RcCommand::kRateThrottle:
        controllers_[mode] = std::make_unique<RateThrottleController>();
        break;
      case tobas::RcCommand::kAngleThrottle:
        controllers_[mode] = std::make_unique<AngleThrottleController>();
        break;
      case tobas::RcCommand::kAccelYaw:
        controllers_[mode] = std::make_unique<AccelYawController>();
        break;
      case tobas::RcCommand::kAccelPitchYaw:
        controllers_[mode] = std::make_unique<AccelPitchYawController>();
        break;
      case tobas::RcCommand::kPosVelYaw:
        controllers_[mode] = std::make_unique<PosVelYawController>();
        break;
      case tobas::RcCommand::kPosVelPitchYaw:
        controllers_[mode] = std::make_unique<PosVelPitchYawController>();
        break;
      case tobas::RcCommand::kAccelRate:
        controllers_[mode] = std::make_unique<AccelRateController>();
        break;
      case tobas::RcCommand::kAccelAngle:
        controllers_[mode] = std::make_unique<AccelAngleController>();
        break;
      case tobas::RcCommand::kPosVelAngle:
        controllers_[mode] = std::make_unique<PosVelAngleController>();
        break;
      case tobas::RcCommand::kSpeedRollDPitch:
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
    TOBAS_ERROR("\"", tobas::kSetArmSrv, "\" is not ready.");
    return;
  }

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  set_arm_sc_->async_send_request(req);
}

void RCTeleopNode::updateWithIdleCommand(const tobas_msgs::RCInput& rcin)
{
  auto idle_rcin = rcin;

  idle_rcin.roll = tobas::kRCInputMid;
  idle_rcin.pitch = tobas::kRCInputMid;
  idle_rcin.yaw = tobas::kRCInputMid;
  idle_rcin.throttle = tobas::kRcInputMin;

  controllers_[tobas::FlightMode::kAcrobat]->update(idle_rcin, *odom_);
}

bool RCTeleopNode::isFlightModeApplicable(tobas::FlightMode mode)
{
  const auto controller_it = controllers_.find(mode);
  if (controller_it == controllers_.end()) {
    TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Invalid flight mode: ", (int)mode);
    return false;
  }
  const auto& controller = controller_it->second;

  if (controller->requirePosition()) {
    const auto max_pos_var = odom_->position_covariance.eigenvalues().real().maxCoeff();
    if (max_pos_var > math::sqr(kPosStddevThresh)) {
      TOBAS_WARN_THROTTLE(
        kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because position estimation is inaccurate.");
      return false;
    }
  }

  if (controller->requireOrientation()) {
    const auto max_rot_var = odom_->orientation_covariance.eigenvalues().real().maxCoeff();
    if (max_rot_var > math::sqr(kRotStddevThresh)) {
      TOBAS_WARN_THROTTLE(
        kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because orientation estimation is inaccurate.");
      return false;
    }
  }

  if (controller->requireLinearVelocity()) {
    const auto max_linvel_var = odom_->velocity_covariance.eigenvalues().real().maxCoeff();
    if (max_linvel_var > math::sqr(kLinVelStddevThresh)) {
      TOBAS_WARN_THROTTLE(
        kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because linear velocity estimation is inaccurate.");
      return false;
    }
  }

  if (controller->requireAngularVelocity()) {
    const auto max_angvel_var = odom_->gyro_covariance.eigenvalues().real().maxCoeff();
    if (max_angvel_var > math::sqr(kAngVelStddevThresh)) {
      TOBAS_WARN_THROTTLE(
        kWarnPeriod, mode2str_.at(mode), " mode cannot be applied because angular velocity estimation is inaccurate.");
      return false;
    }
  }

  return true;
}

void RCTeleopNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void RCTeleopNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RCTeleopNode::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  health_ = health;
}

void RCTeleopNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

void RCTeleopNode::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  // 通信チェックタイマーをリセット
  rcin_timeout_timer_->reset();

  switch (stage_) {
    case kCheckPrerequisites: {
      if (!odom_) {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for odometry.");
        break;
      }
      if (!arming_) {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for arming status.");
        break;
      }
      if (!health_) {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Warting for vehicle health status.");
        break;
      }
      if (!landed_) {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Warting for landing status.");
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
        // プロポを起動した瞬間ディスアームされるのを防ぐため，Killスイッチがオンの時はRC制御モードには移行しない．
        if (rcin->kill) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Cannot switch to RC control mode because the kill switch is on.");
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        stage_ = kWaitForEnable;
        break;
      }

      // アームコマンドが入力されている場合
      if (
        std::max(abs(rcin->roll), abs(rcin->pitch)) < kArmThrotThresh &&
        rcin->yaw < tobas::kRcInputMin + kArmThrotThresh && rcin->throttle < tobas::kRcInputMin + kArmThrotThresh) {
        // アームコマンドが一定時間維持されていれば一度アームをリクエスト
        if ((rcin->header.stamp - t_arm_start_).seconds() > kArmDuration) {
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

      // RC入力が有効かつ実行可能な飛行モードだったら次のステージへ
      if (rcin->enable && isFlightModeApplicable(rcin->mode)) {
        controllers_.at(rcin->mode)->reset(*odom_);
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
        controllers_[rcin->mode]->reset(*odom_);
        cur_mode_ = rcin->mode;
        TOBAS_INFO("Flight mode changed to \"", mode2str_.at(rcin->mode), "\".");
      }

      if (landed_->data && rcin->throttle < tobas::kRcInputMin + kArmThrotThresh) {  // 地上でゼロスロットルの場合
        // 安全のためアイドルコマンドを送信
        updateWithIdleCommand(*rcin);

        // さらにディスアームコマンドの場合
        if (std::max(abs(rcin->roll), abs(rcin->pitch)) < kArmThrotThresh && rcin->yaw > tobas::kRcInputMax - kArmThrotThresh) {
          TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Disarm commanded.");
          if ((rcin->header.stamp - t_disarm_start_).seconds() > kDisarmDuration) {  // 一定時間維持されていればリクエスト
            TOBAS_INFO("Disarming rotors...");
            requestArmingRotors(false);
            t_disarm_start_ = rcin->header.stamp;
          }
          break;  // ディスアームコマンドの開始時刻を更新せずに抜ける
        }
      }
      else {  // それ以外は普通にコマンド送信
        controllers_[cur_mode_]->update(*rcin, *odom_);
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

void RCTeleopNode::rcInputTimeoutTimerCb()
{
  if (stage_ != kCheckPrerequisites) {
    // 通信が切れる前のコマンドが残らないようにステージを最初に戻す
    TOBAS_WARN("Radio link was lost.");
    stage_ = kCheckPrerequisites;
  }
}
}  // namespace tobas_rc_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(tobas_rc_teleop::RCTeleopNode)
