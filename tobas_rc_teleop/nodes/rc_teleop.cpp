#include <eigen3/Eigen/Eigen>

#include <tobas_math/core.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include "../include/tobas_rc_teleop/common.hpp"
#include "../include/tobas_rc_teleop/rate_throttle.hpp"
#include "../include/tobas_rc_teleop/angle_throttle.hpp"
#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/pose_twist_accel.hpp"
#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"

using namespace std;
using namespace tobas_msgs::msg;
using namespace tobas_msgs::srv;

namespace tobas_rc_teleop
{
class RCTeleopNode : public tobas::BaseNode
{
  static constexpr double kArmThrotThresh = 0.1;  // [-]
  static constexpr double kArmDuration = 5.;      // [s]
  static constexpr double kDisarmDuration = 2.;   // [s]

  static constexpr double kPosStddevThresh = 3.;           // [m]
  static constexpr double kRotStddevThresh = M_PI / 12;    // [rad]
  static constexpr double kLinVelStddevThresh = 1.;        // [m/s]
  static constexpr double kAngVelStddevThresh = M_PI / 3;  // [rad/s]

  static constexpr double kArmCommandInfoPeriod = 1.;  // [s]
  static constexpr double kModeChangeWarnPeriod = 1.;  // [s]

  using self = RCTeleopNode;
  using super = tobas::BaseNode;

public:
  explicit RCTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum stage_t
  {
    CHECK_PREREQUISITES,
    WAIT_FOR_ARMING,
    FIRST_COMMAND,
    RUNNING,
  } stage_ = CHECK_PREREQUISITES;

  const map<uint8_t, const char*> mode2str_{
    { tobas::flight_mode_t::ACROBAT_MODE, "Acrobat" },
    { tobas::flight_mode_t::STABILIZE_MODE, "Stabilize" },
    { tobas::flight_mode_t::LOITER_MODE, "Loiter" },
  };

  // rosparams
  array<tobas::rc_command_t, tobas::kNumFlightModes> modes_;

  // Mutables
  uint8_t cur_mode_;
  rclcpp::Time t_arm_start_;
  rclcpp::Time t_disarm_start_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  // Controllers
  array<unique_ptr<BaseController>, tobas::kNumFlightModes> controllers_;

  // PubSub
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;
  ros2::SubscriberPtr<RCInput> rcin_sub_;

  // Service
  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  void getStaticRosParams();
  void initializeControllers();
  void requestArmingRotors(bool arming);

  bool isArmCommand(const tobas_msgs::msg::RCInput& rcin);
  bool isDisarmCommand(const tobas_msgs::msg::RCInput& rcin);

  bool isFlightModeApplicable(uint8_t mode);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);
  void rcInputCb(const RCInput::ConstSharedPtr& rcin);
};

RCTeleopNode::RCTeleopNode(const rclcpp::NodeOptions& options) : super("rc_teleop", options)
{
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
  modes_[tobas::flight_mode_t::ACROBAT_MODE] = static_cast<tobas::rc_command_t>(getIntParam("acrobat_mode"));
  modes_[tobas::flight_mode_t::STABILIZE_MODE] = static_cast<tobas::rc_command_t>(getIntParam("stabilize_mode"));
  modes_[tobas::flight_mode_t::LOITER_MODE] = static_cast<tobas::rc_command_t>(getIntParam("loiter_mode"));
}

void RCTeleopNode::initializeControllers()
{
  // 各フライトモードに対応するコントローラを設定
  for (size_t i = 0; i < tobas::kNumFlightModes; ++i)
  {
    switch (modes_[i])
    {
      case tobas::rc_command_t::RATE_THROTTLE:
        controllers_[i] = std::make_unique<RateThrottleController>();
        break;
      case tobas::rc_command_t::ANGLE_THROTTLE:
        controllers_[i] = std::make_unique<AngleThrottleController>();
        break;
      case tobas::rc_command_t::POS_VEL_ACC_YAW:
        controllers_[i] = std::make_unique<PosVelAccYawController>();
        break;
      case tobas::rc_command_t::POSE_TWIST_ACCEL:
        controllers_[i] = std::make_unique<PoseTwistAccelController>();
        break;
      case tobas::rc_command_t::SPEED_ROLL_DPITCH:
        controllers_[i] = std::make_unique<SpeedRollDeltaPitchController>();
        break;
      default:
        TOBAS_EXIT("Invalid flight mode: ", (int)modes_[i]);
    }

    controllers_[i]->initialize(this);
  }
}

void RCTeleopNode::requestArmingRotors(bool arming)
{
  if (!set_arm_sc_->service_is_ready())
  {
    TOBAS_ERROR("\"", tobas::kSetArmSrv, "\" is not ready.");
    return;
  }

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  set_arm_sc_->async_send_request(req);
}

bool RCTeleopNode::isArmCommand(const tobas_msgs::msg::RCInput& rcin)
{
  return abs(rcin.roll) < kArmThrotThresh && abs(rcin.pitch) < kArmThrotThresh && rcin.yaw < -1 + kArmThrotThresh
         && rcin.throttle < -1 + kArmThrotThresh;
}

bool RCTeleopNode::isDisarmCommand(const tobas_msgs::msg::RCInput& rcin)
{
  return abs(rcin.roll) < kArmThrotThresh && abs(rcin.pitch) < kArmThrotThresh && rcin.yaw > 1 - kArmThrotThresh
         && rcin.throttle < -1 + kArmThrotThresh;
}

bool RCTeleopNode::isFlightModeApplicable(uint8_t mode)
{
  const auto& controller = controllers_.at(mode);

  if (controller->requirePosition())
  {
    const auto max_pos_var = odom_->position_covariance.eigenvalues().real().maxCoeff();
    if (max_pos_var > math::sqr(kPosStddevThresh))
    {
      TOBAS_WARN_THROTTLE(
        kModeChangeWarnPeriod, mode2str_.at(mode), " mode cannot be appied because position estimation is innacurate.");
      return false;
    }
  }

  if (controller->requireOrientation())
  {
    const auto max_rot_var = odom_->orientation_covariance.eigenvalues().real().maxCoeff();
    if (max_rot_var > math::sqr(kRotStddevThresh))
    {
      TOBAS_WARN_THROTTLE(
        kModeChangeWarnPeriod, mode2str_.at(mode),
        " mode cannot be appied because orientation estimation is innacurate.");
      return false;
    }
  }

  if (controller->requireLinearVelocity())
  {
    const auto max_linvel_var = odom_->velocity_covariance.eigenvalues().real().maxCoeff();
    if (max_linvel_var > math::sqr(kLinVelStddevThresh))
    {
      TOBAS_WARN_THROTTLE(
        kModeChangeWarnPeriod, mode2str_.at(mode),
        " mode cannot be appied because linear velocity estimation is innacurate.");
      return false;
    }
  }

  if (controller->requireAngularVelocity())
  {
    const auto max_angvel_var = odom_->gyro_covariance.eigenvalues().real().maxCoeff();
    if (max_angvel_var > math::sqr(kAngVelStddevThresh))
    {
      TOBAS_WARN_THROTTLE(
        kModeChangeWarnPeriod, mode2str_.at(mode),
        " mode cannot be appied because angular velocity estimation is innacurate.");
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

void RCTeleopNode::rcInputCb(const RCInput::ConstSharedPtr& rcin)
{
  // RC入力が有効化されてなければステージを初期化して終了
  if (!rcin->enable)
  {
    stage_ = CHECK_PREREQUISITES;
    return;
  }

  switch (stage_)
  {
    case CHECK_PREREQUISITES:
    {
      if (odom_ == nullptr)
      {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for odometry.");
        break;
      }
      if (arming_ == nullptr)
      {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for arming status.");
        break;
      }
      if (prearm_check_ == nullptr)
      {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Warting for pre-arm check status.");
        break;
      }

      t_arm_start_ = rcin->header.stamp;
      stage_ = WAIT_FOR_ARMING;
      break;
    }

    case WAIT_FOR_ARMING:
    {
      // アームされていればコマンド送信開始
      if (arming_->data)
      {
        stage_ = FIRST_COMMAND;
        break;
      }

      // アームコマンドが一定時間維持されていれば一度アームをリクエスト
      if ((rcin->header.stamp - t_arm_start_).seconds() > kArmDuration)
      {
        TOBAS_INFO("Requesting arming rotors...");
        requestArmingRotors(true);
        t_arm_start_ = rcin->header.stamp;
        break;
      }

      // アームコマンドでかつPre-Arm Checkにクリアしているなら時刻を初期化せず継続
      if (isArmCommand(*rcin))
      {
        TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Arm commanded.");

        if (prearm_check_->ok)
        {
          break;
        }
        else
        {
          TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Cannot arm because pre-arm check failed.");
          t_arm_start_ = rcin->header.stamp;
        }
      }

      t_arm_start_ = rcin->header.stamp;
      break;
    }

    case FIRST_COMMAND:
    {
      const auto& new_mode = rcin->mode;
      if (new_mode >= tobas::kNumFlightModes)
      {
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Invalid flight mode: ", (int)new_mode);
        break;
      }

      if (!isFlightModeApplicable(new_mode))
        break;

      controllers_[new_mode]->reset(*odom_);
      cur_mode_ = new_mode;
      TOBAS_INFO("First flight mode is set to \"", mode2str_.at(new_mode), "\".");

      t_disarm_start_ = rcin->header.stamp;
      stage_ = RUNNING;
      break;
    }

    case RUNNING:
    {
      // ディスアームされていればステージをリセット
      if (!arming_->data)
      {
        stage_ = WAIT_FOR_ARMING;
        break;
      }

      // ディスアームコマンドの場合
      if (isDisarmCommand(*rcin))
      {
        TOBAS_INFO_THROTTLE(kArmCommandInfoPeriod, "Disarm commanded.");

        // ディスアームコマンドが一定時間維持されていればリクエスト
        if ((rcin->header.stamp - t_disarm_start_).seconds() > kDisarmDuration)
        {
          TOBAS_INFO("Requesting disarming rotors...");
          requestArmingRotors(false);
          t_disarm_start_ = rcin->header.stamp;
        }
        break;
      }

      // ディスアームコマンドの開始時刻を更新
      t_disarm_start_ = rcin->header.stamp;

      // フライトモードを取得
      const auto& new_mode = rcin->mode;
      if (new_mode >= tobas::kNumFlightModes)
      {
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Invalid flight mode.");
        break;
      }

      // フライトモードの変更があった場合，適用可能な場合に限り変更する．
      // 適用できない場合は前のフライトモードを継続する．
      if (new_mode != cur_mode_ && isFlightModeApplicable(new_mode))
      {
        controllers_[new_mode]->reset(*odom_);
        cur_mode_ = new_mode;
        TOBAS_INFO("Flight mode changed to \"", mode2str_.at(new_mode), "\".");
        break;
      }

      // コマンド送信
      controllers_[cur_mode_]->update(*rcin, *odom_);

      break;
    }

    default:
    {
      TOBAS_ERROR("Invalid stage: ", (int)stage_);
      break;
    }
  }
}
}  // namespace tobas_rc_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(tobas_rc_teleop::RCTeleopNode)
