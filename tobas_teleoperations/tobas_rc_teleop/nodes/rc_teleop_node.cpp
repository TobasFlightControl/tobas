#include <tobas_math/core.hpp>
#include <tobas_std_tools/string.hpp>
#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/PositionYaw.hpp>
#include <tobas_msgs/RollPitchYawThrottle.hpp>
#include <tobas_msgs/PoseTwistAccelCommand.hpp>
#include <tobas_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs/srv/get_arm.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include "../include/tobas_rc_teleop/common.hpp"
#include "../include/tobas_rc_teleop/program_mode.hpp"
#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/position_yaw.hpp"
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
  static constexpr auto kArmFailRetryInterval = 1s;

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
    FIRST_COMMAND,
    RUNNING,
  } stage_ = CHECK_PREREQUISITES;

  const map<uint8_t, const char*> mode2str_{
    { tobas::kFlightModeProgram, "Program" },
    { tobas::kFlightModeStabilize, "Stabilize" },
    { tobas::kFlightModeAcrobat, "Acrobat" },
  };

  // rosparams
  array<string, tobas::kNumFlightModes> modes_;

  // Mutables
  uint8_t last_mode_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  // Controllers
  array<unique_ptr<BaseController>, tobas::kNumFlightModes> controllers_;

  // PubSub
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<RCInput> rcin_sub_;

  void getStaticRosParams();
  void initializeControllers();
  bool isRotorsArmed();
  bool requestArmingRotors();
  bool requestDisarmingRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void rcInputCb(const RCInput::ConstSharedPtr& rcin);
};

RCTeleopNode::RCTeleopNode(const rclcpp::NodeOptions& options) : super("rc_teleop", options)
{
  getStaticRosParams();
  initializeControllers();

  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  rcin_sub_ = createSubscriber(tobas::kRcInputTopic, &self::rcInputCb, this);
}

void RCTeleopNode::getStaticRosParams()
{
  modes_[tobas::kFlightModeStabilize] = getStringParam("stabilize_mode");
  modes_[tobas::kFlightModeAcrobat] = getStringParam("acrobat_mode");
}

void RCTeleopNode::initializeControllers()
{
  // プログラムモードのダミーコントローラを設定
  controllers_[tobas::kFlightModeProgram] = std::make_unique<ProgramModeController>();

  // その他の飛行モードのコントローラを設定
  for (size_t i = 1; i < tobas::kNumFlightModes; ++i)
  {
    if (modes_[i] == "")
      controllers_[i] = std::make_unique<ProgramModeController>();
    else if (modes_[i] == tobas_std::split(rosidl_generator_traits::name<PosVelAccYaw>(), '/').back())
      controllers_[i] = std::make_unique<PosVelAccYawController>();
    else if (modes_[i] == tobas_std::split(rosidl_generator_traits::name<PositionYaw>(), '/').back())
      controllers_[i] = std::make_unique<PositionYawController>();
    else if (modes_[i] == tobas_std::split(rosidl_generator_traits::name<RollPitchYawThrottle>(), '/').back())
      controllers_[i] = std::make_unique<RollPitchYawThrottleController>();
    else if (modes_[i] == tobas_std::split(rosidl_generator_traits::name<PoseTwistAccelCommand>(), '/').back())
      controllers_[i] = std::make_unique<PoseTwistAccelController>();
    else if (modes_[i] == tobas_std::split(rosidl_generator_traits::name<SpeedRollDeltaPitch>(), '/').back())
      controllers_[i] = std::make_unique<SpeedRollDeltaPitchController>();
    else
    {
      TOBAS_ERROR("Invalid flight mode: ", modes_[i], ". The RC command for this mode will not be published.");
      controllers_[i] = std::make_unique<ProgramModeController>();
    }

    controllers_[i]->initialize(this);
  }
}

bool RCTeleopNode::isRotorsArmed()
{
  ros2::SimpleServiceClient<tobas_msgs::srv::GetArm> sc(shared_from_this(), tobas::kGetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::GetArm::Request>();
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to get arming status.");
    return false;
  }

  const auto& res = sc.getResponse();
  return res->arming;
}

bool RCTeleopNode::requestArmingRotors()
{
  ros2::SimpleServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = true;
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kSetArmSrv, "\" service.");
    return false;
  }

  const auto& res = sc.getResponse();
  if (!res->success)
  {
    TOBAS_ERROR("Failed to arm rotors: ", res->message);
    return false;
  }

  return true;
}

bool RCTeleopNode::requestDisarmingRotors()
{
  ros2::SimpleServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kSetArmSrv, "\" service.");
    return false;
  }

  const auto& res = sc.getResponse();
  if (!res->success)
  {
    TOBAS_ERROR("Failed to disarm rotors: ", res->message);
    return false;
  }

  return true;
}

void RCTeleopNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (odom->status != Odometry::NO_ERROR)
    return;

  odom_ = odom;
}

void RCTeleopNode::rcInputCb(const RCInput::ConstSharedPtr& rcin)
{
  switch (stage_)
  {
    case CHECK_PREREQUISITES:
    {
      if (odom_ != nullptr)
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
        TOBAS_INFO_THROTTLE(kInfoPeriod, "Please start with the transmitter's E-Stop toggle ON.");
      }
      break;
    }

    case ESTOP_ON:
    {
      // E-StopがONのままならスキップ
      if (rcin->e_stop)
        break;

      // 既にアームされていたら，即コマンドを送信開始 (緊急的に制御を奪いたいときなど)
      if (isRotorsArmed())
      {
        stage_ = FIRST_COMMAND;
        break;
      }

      // アームされていなければ，スロットルレバーを確認してアームする
      if (rcin->throttle > tobas::kRCInputMin + kInitThrottleMargin)
      {
        TOBAS_WARN_THROTTLE(kWarnPeriod, "Please lower the throttle lever to the bottom before turning off E-Stop.");
        break;
      }
      if (!requestArmingRotors())
      {
        rclcpp::sleep_for(kArmFailRetryInterval);
        break;
      }

      // 問題なければコマンドを送信開始
      stage_ = FIRST_COMMAND;
      break;
    }

    case FIRST_COMMAND:
    {
      const auto& cur_mode = rcin->mode;
      if (cur_mode >= tobas::kNumFlightModes)
      {
        TOBAS_ERROR_THROTTLE(kErrorPeriod, "Invalid flight mode.");
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
        requestDisarmingRotors();
        stage_ = ESTOP_ON;
      }

      const auto& cur_mode = rcin->mode;
      if (cur_mode >= tobas::kNumFlightModes)
      {
        TOBAS_ERROR_THROTTLE(kErrorPeriod, "Invalid flight mode.");
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
