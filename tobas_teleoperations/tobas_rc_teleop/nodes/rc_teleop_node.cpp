#include <std_msgs/msg/bool.hpp>

#include <tobas_math/core.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>

#include "../include/tobas_rc_teleop/common.hpp"
#include "../include/tobas_rc_teleop/program_mode.hpp"
#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/rpy_throttle.hpp"
#include "../include/tobas_rc_teleop/pose_twist_accel.hpp"
#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"

using namespace std;
using namespace tobas_msgs::msg;
using namespace tobas_msgs::srv;

namespace tobas_rc_teleop
{
class RCTeleopNode : public tobas::BaseNode
{
  static constexpr double kInitThrottleMargin = 0.05;
  static constexpr double kRequestArmingInterval = 1.;  // [s]

  using self = RCTeleopNode;
  using super = tobas::BaseNode;

public:
  explicit RCTeleopNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum stage_t
  {
    CHECK_PREREQUISITES,
    WAIT_FOR_ESTOP,
    ESTOP_ON,
    WAIT_FOR_ARMING,
    FIRST_COMMAND,
    RUNNING,
  } stage_ = CHECK_PREREQUISITES;

  const map<uint8_t, const char*> mode2str_{
    { tobas::flight_mode_t::PROGRAM_MODE, "Program" },
    { tobas::flight_mode_t::STABILIZE_MODE, "Stabilize" },
    { tobas::flight_mode_t::ACROBAT_MODE, "Acrobat" },
  };

  // rosparams
  array<tobas::rc_command_t, tobas::kNumFlightModes> modes_;

  // Mutables
  uint8_t last_mode_;
  rclcpp::Time t_last_arming_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;

  // Controllers
  array<unique_ptr<BaseController>, tobas::kNumFlightModes> controllers_;

  // PubSub
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<RCInput> rcin_sub_;

  // Service
  ros2::ServiceClientPtr<tobas_msgs::srv::SetArm> set_arm_sc_;

  void getStaticRosParams();
  void initializeControllers();
  void requestArmingRotors(bool arming);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void rcInputCb(const RCInput::ConstSharedPtr& rcin);
};

RCTeleopNode::RCTeleopNode(const rclcpp::NodeOptions& options) : super("rc_teleop", options)
{
  getStaticRosParams();
  initializeControllers();

  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  rcin_sub_ = createSubscriber(tobas::kRcInputTopic, &self::rcInputCb, this);

  set_arm_sc_ = create_client<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv);
}

void RCTeleopNode::getStaticRosParams()
{
  modes_[tobas::flight_mode_t::STABILIZE_MODE] =
    static_cast<tobas::rc_command_t>(getIntParam("stabilize_mode", tobas::rc_command_t::PROGRAM));
  modes_[tobas::flight_mode_t::ACROBAT_MODE] =
    static_cast<tobas::rc_command_t>(getIntParam("acrobat_mode", tobas::rc_command_t::PROGRAM));
}

void RCTeleopNode::initializeControllers()
{
  // プログラムモードのダミーコントローラを設定
  controllers_[tobas::flight_mode_t::PROGRAM_MODE] = std::make_unique<ProgramModeController>();

  // その他の飛行モードのコントローラを設定
  for (size_t i = 1; i < tobas::kNumFlightModes; ++i)
  {
    switch (modes_[i])
    {
      case tobas::rc_command_t::PROGRAM:
        controllers_[i] = std::make_unique<ProgramModeController>();
        break;
      case tobas::rc_command_t::POS_VEL_ACC_YAW:
        controllers_[i] = std::make_unique<PosVelAccYawController>();
        break;
      case tobas::rc_command_t::ROLL_PITCH_YAW_THROTTLE:
        controllers_[i] = std::make_unique<RollPitchYawThrottleController>();
        break;
      case tobas::rc_command_t::POSE_TWIST_ACCEL:
        controllers_[i] = std::make_unique<PoseTwistAccelController>();
        break;
      case tobas::rc_command_t::SPEED_ROLL_DPITCH:
        controllers_[i] = std::make_unique<SpeedRollDeltaPitchController>();
        break;
      default:
        TOBAS_ERROR("Invalid flight mode. The RC command for this mode will not be published.");
        controllers_[i] = std::make_unique<ProgramModeController>();
        break;
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

void RCTeleopNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void RCTeleopNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RCTeleopNode::rcInputCb(const RCInput::ConstSharedPtr& rcin)
{
  switch (stage_)
  {
    case CHECK_PREREQUISITES:
    {
      if (odom_ == nullptr)
      {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for odometry.");
        break;
      }
      if (odom_->status != Odometry::NO_ERROR)
      {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "State estimation problem.");
        break;
      }
      if (arming_ == nullptr)
      {
        TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Waiting for arming status.");
        break;
      }

      stage_ = WAIT_FOR_ESTOP;
      break;
    }

    case WAIT_FOR_ESTOP:
    {
      if (rcin->e_stop)
      {
        TOBAS_INFO("RC transmitter is ready. "
                   "To start control, set the E-Stop toggle OFF "
                   "with the throttle lever lowered to the bottom.");
        stage_ = ESTOP_ON;
      }
      else
      {
        TOBAS_INFO_THROTTLE(tobas::kTypicalInfoPeriod, "Please start with the transmitter's E-Stop toggle ON.");
      }
      break;
    }

    case ESTOP_ON:
    {
      // E-STOPがONのままならスキップ
      if (rcin->e_stop)
        break;

      // E-STOPがOFF且つアームされていればコマンド送信開始
      if (arming_->data)
      {
        stage_ = FIRST_COMMAND;
        break;
      }

      // スロットルレバーが下がっていないとアームできない
      if (rcin->throttle > tobas::kRCInputMin + kInitThrottleMargin)
      {
        TOBAS_WARN_THROTTLE(
          tobas::kTypicalWarnPeriod, "Please lower the throttle lever to the bottom before turning off E-Stop.");
        break;
      }

      // スロットルレバーが下がっているなら一定時間間隔でアームをリクエストして終了
      if (t_last_arming_.nanoseconds() == 0 || (get_clock()->now() - t_last_arming_).seconds() > kRequestArmingInterval)
      {
        TOBAS_INFO("Starting rotors.");
        requestArmingRotors(true);
        t_last_arming_ = get_clock()->now();
      }

      break;
    }

    case FIRST_COMMAND:
    {
      const auto& cur_mode = rcin->mode;
      if (cur_mode >= tobas::kNumFlightModes)
      {
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Invalid flight mode.");
        return;
      }

      controllers_[cur_mode]->reset(*odom_);
      last_mode_ = cur_mode;
      TOBAS_INFO("First flight mode is set to \"", mode2str_.at(cur_mode), "\".");

      stage_ = RUNNING;
      break;
    }

    case RUNNING:
    {
      // E-Stopがオンになったら非常停止
      if (rcin->e_stop)
      {
        TOBAS_WARN("Stopping rotors.");
        requestArmingRotors(false);
        stage_ = ESTOP_ON;
      }

      const auto& cur_mode = rcin->mode;
      if (cur_mode >= tobas::kNumFlightModes)
      {
        TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Invalid flight mode.");
        return;
      }

      if (cur_mode != last_mode_)
      {
        last_mode_ = cur_mode;
        controllers_[cur_mode]->reset(*odom_);
        TOBAS_INFO("Flight mode changed to \"", mode2str_.at(cur_mode), "\".");
        break;
      }

      controllers_[cur_mode]->update(*rcin, *odom_);
      break;
    }

    default:
    {
      TOBAS_ERROR("Invalid stage: ", static_cast<int>(stage_));
      break;
    }
  }
}
}  // namespace tobas_rc_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(tobas_rc_teleop::RCTeleopNode)
