#include <eigen3/Eigen/Eigen>
#include <magic_enum/magic_enum.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_constants/rc_command.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include "../include/tobas_rc_teleop/rate_throttle.hpp"
#include "../include/tobas_rc_teleop/angle_throttle.hpp"
#include "../include/tobas_rc_teleop/accel_yaw.hpp"
#include "../include/tobas_rc_teleop/pos_vel_yaw.hpp"
#include "../include/tobas_rc_teleop/accel_rate.hpp"
#include "../include/tobas_rc_teleop/accel_angle.hpp"
#include "../include/tobas_rc_teleop/pos_vel_angle.hpp"
#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"

using namespace std;

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

  using self = RCTeleopNode;
  using super = tobas::BaseNode;

public:
  explicit RCTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum stage_t
  {
    // Pre-Arm
    CHECK_PREREQUISITES,
    WAIT_FOR_ARMING,

    // Post-Arm
    WAIT_FOR_ENABLE,
    WAIT_FOR_THROTTLE,
    RUNNING,
  } stage_ = CHECK_PREREQUISITES;

  const map<tobas::flight_mode_t, const char*> mode2str_{
    { tobas::flight_mode_t::ACROBAT, "Acrobat" },
    { tobas::flight_mode_t::STABILIZE, "Stabilize" },
    { tobas::flight_mode_t::LOITER, "Loiter" },
  };

  // rosparams
  map<tobas::flight_mode_t, tobas::rc_command_t> modes_;

  // Mutables
  tobas::flight_mode_t cur_mode_;
  rclcpp::Time t_arm_start_;
  rclcpp::Time t_disarm_start_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  // Controllers
  map<tobas::flight_mode_t, unique_ptr<BaseController>> controllers_;

  // PubSub
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;

  // Service
  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  void getStaticRosParams();
  void initializeControllers();
  void requestArmingRotors(bool arming);
  void updateWithIdleCommand(const tobas_msgs::RCInput& rcin);

  bool isArmCommand(const tobas_msgs::RCInput& rcin);
  bool isDisarmCommand(const tobas_msgs::RCInput& rcin);

  bool isFlightModeApplicable(tobas::flight_mode_t mode);

  /* アーム後の共通処理．返り値がtrueならば離脱． */
  bool postArmCommonProcess(const tobas_msgs::RCInput& rcin);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};

RCTeleopNode::RCTeleopNode(const rclcpp::NodeOptions& options) : super(tobas::node::kRcTeleop, options)
{
  TOBAS_CHECK(mode2str_.size() == magic_enum::enum_count<tobas::flight_mode_t>());

  getStaticRosParams();
  initializeControllers();

  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);
  rcin_sub_ = createSubscriber(tobas::kRcInputTopic, &self::rcInputCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
}

void RCTeleopNode::getStaticRosParams()
{
  for (const auto& mode : magic_enum::enum_values<tobas::flight_mode_t>()) {
    modes_[mode];
  }

  TOBAS_CHECK(tobas::enumFromText(getStringParam("acrobat_mode"), modes_.at(tobas::flight_mode_t::ACROBAT)));
  TOBAS_CHECK(tobas::enumFromText(getStringParam("stabilize_mode"), modes_.at(tobas::flight_mode_t::STABILIZE)));
  TOBAS_CHECK(tobas::enumFromText(getStringParam("loiter_mode"), modes_.at(tobas::flight_mode_t::LOITER)));
}

void RCTeleopNode::initializeControllers()
{
  // 各フライトモードに対応するコントローラを設定
  for (const auto& [mode, cmd] : modes_) {
    switch (cmd) {
      case tobas::rc_command_t::RATE_THROTTLE:
        controllers_[mode] = std::make_unique<RateThrottleController>();
        break;
      case tobas::rc_command_t::ANGLE_THROTTLE:
        controllers_[mode] = std::make_unique<AngleThrottleController>();
        break;
      case tobas::rc_command_t::ACCEL_YAW:
        controllers_[mode] = std::make_unique<AccelYawController>();
        break;
      case tobas::rc_command_t::POS_VEL_YAW:
        controllers_[mode] = std::make_unique<PosVelYawController>();
        break;
      case tobas::rc_command_t::ACCEL_RATE:
        controllers_[mode] = std::make_unique<AccelRateController>();
        break;
      case tobas::rc_command_t::ACCEL_ANGLE:
        controllers_[mode] = std::make_unique<AccelAngleController>();
        break;
      case tobas::rc_command_t::POS_VEL_ANGLE:
        controllers_[mode] = std::make_unique<PosVelAngleController>();
        break;
      case tobas::rc_command_t::SPEED_ROLL_DPITCH:
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

bool RCTeleopNode::postArmCommonProcess(const tobas_msgs::RCInput& rcin)
{
  // ディスアームされていればステージをリセット
  if (!arming_->data) {
    t_arm_start_ = rcin.header.stamp;
    stage_ = CHECK_PREREQUISITES;
    return true;
  }

  // Killスイッチがオンならば即ディスアーム
  if (rcin.kill) {
    TOBAS_WARN_THROTTLE(kWarnPeriod, "The kill switch has been activated. Forcing disarm.");
    requestArmingRotors(false);
    return true;
  }

  // Enableスイッチがオフならば待機モードに戻る
  if (!rcin.enable) {
    if (stage_ != WAIT_FOR_ENABLE) {
      TOBAS_INFO("RC control is disabled.");
      stage_ = WAIT_FOR_ENABLE;
    }

    t_disarm_start_ = rcin.header.stamp;
    return true;
  }

  // ディスアームコマンドの場合
  if (isDisarmCommand(rcin)) {
    TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Disarm commanded.");

    // 安全のためアイドルコマンドを送信
    updateWithIdleCommand(rcin);

    // ディスアームコマンドが一定時間維持されていればリクエスト
    if ((rcin.header.stamp - t_disarm_start_).seconds() > kDisarmDuration) {
      TOBAS_INFO("Requesting disarming rotors...");
      requestArmingRotors(false);
      t_disarm_start_ = rcin.header.stamp;
    }

    return true;
  }

  t_disarm_start_ = rcin.header.stamp;
  return false;
}

void RCTeleopNode::updateWithIdleCommand(const tobas_msgs::RCInput& rcin)
{
  auto idle_rcin = rcin;

  idle_rcin.roll = tobas::kRCInputMid;
  idle_rcin.pitch = tobas::kRCInputMid;
  idle_rcin.yaw = tobas::kRCInputMid;
  idle_rcin.throttle = tobas::kRcInputMin;

  controllers_[cur_mode_]->update(idle_rcin, *odom_);
}

bool RCTeleopNode::isArmCommand(const tobas_msgs::RCInput& rcin)
{
  return abs(rcin.roll) < kArmThrotThresh && abs(rcin.pitch) < kArmThrotThresh &&
         rcin.yaw < tobas::kRcInputMin + kArmThrotThresh && rcin.throttle < tobas::kRcInputMin + kArmThrotThresh;
}

bool RCTeleopNode::isDisarmCommand(const tobas_msgs::RCInput& rcin)
{
  return abs(rcin.roll) < kArmThrotThresh && abs(rcin.pitch) < kArmThrotThresh &&
         rcin.yaw > tobas::kRcInputMax - kArmThrotThresh && rcin.throttle < tobas::kRcInputMin + kArmThrotThresh;
}

bool RCTeleopNode::isFlightModeApplicable(tobas::flight_mode_t mode)
{
  const auto& controller = controllers_.at(mode);

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

void RCTeleopNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void RCTeleopNode::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  switch (stage_) {
    case CHECK_PREREQUISITES: {
      if (!odom_) {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for odometry.");
        break;
      }
      if (!arming_) {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for arming status.");
        break;
      }
      if (!prearm_check_) {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Warting for pre-arm check status.");
        break;
      }

      t_arm_start_ = rcin->header.stamp;
      stage_ = WAIT_FOR_ARMING;
      break;
    }

    case WAIT_FOR_ARMING: {
      // アームされていれば次のステージに以降
      // プログラムモードから制御を奪う場合のために，アームコマンドの確認の前に現在のアーム状態の確認を行う．
      if (arming_->data) {
        // プロポを起動した瞬間ディスアームされるのを防ぐため，Killスイッチがオンの時はRC制御モードには移行しない．
        if (rcin->kill) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Cannot switch to RC control mode because the kill switch is on.");
          t_arm_start_ = rcin->header.stamp;
          break;
        }

        stage_ = WAIT_FOR_ENABLE;
        break;
      }

      // アームコマンドが入力されている場合
      if (isArmCommand(*rcin)) {
        // アームコマンドが一定時間維持されていれば一度アームをリクエスト
        if ((rcin->header.stamp - t_arm_start_).seconds() > kArmDuration) {
          TOBAS_INFO("Requesting arming rotors...");
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

        if (!prearm_check_->ok) {
          TOBAS_WARN_THROTTLE(kWarnPeriod, "Cannot arm because pre-arm check failed.");
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

    case WAIT_FOR_ENABLE: {
      if (postArmCommonProcess(*rcin)) {
        break;
      }

      if (!rcin->enable) {
        break;
      }

      if (!modes_.contains(rcin->mode)) {
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Invalid flight mode: ", (int)rcin->mode);
        break;
      }

      if (!isFlightModeApplicable(rcin->mode)) {
        break;
      }

      controllers_.at(rcin->mode)->reset(*odom_);
      cur_mode_ = rcin->mode;
      TOBAS_INFO("First flight mode is set to \"", mode2str_.at(rcin->mode), "\".");

      stage_ = WAIT_FOR_THROTTLE;
      break;
    }

    case WAIT_FOR_THROTTLE: {
      if (postArmCommonProcess(*rcin)) {
        break;
      }

      if (rcin->throttle > tobas::kRcInputMin + kArmThrotThresh) {
        // スロットルが上がっていればコマンド送信開始
        TOBAS_INFO("The throttle lever has risen, starting RC command transmission.");
        t_disarm_start_ = rcin->header.stamp;
        stage_ = RUNNING;
      }
      else {
        // アーム直後でスロットルが下がったままならばアイドルコマンドを送信
        TOBAS_INFO_THROTTLE(tobas::kTypicalInfoPeriod, "The throttle lever is lowered, sending a idle command.");
        updateWithIdleCommand(*rcin);
      }

      break;
    }

    case RUNNING: {
      if (postArmCommonProcess(*rcin)) {
        break;
      }

      // フライトモードを取得
      if (!modes_.contains(rcin->mode)) {
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Invalid flight mode.");
        break;
      }

      // フライトモードの変更があった場合，適用可能な場合に限り変更する．
      // 適用できない場合は前のフライトモードを継続する．
      if (rcin->mode != cur_mode_ && isFlightModeApplicable(rcin->mode)) {
        controllers_[rcin->mode]->reset(*odom_);
        cur_mode_ = rcin->mode;
        TOBAS_INFO("Flight mode changed to \"", mode2str_.at(rcin->mode), "\".");
        break;
      }

      // コマンド送信
      controllers_[cur_mode_]->update(*rcin, *odom_);

      break;
    }

    default: {
      TOBAS_ERROR("Invalid stage: ", (int)stage_);
      break;
    }
  }
}
}  // namespace tobas_rc_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(tobas_rc_teleop::RCTeleopNode)
