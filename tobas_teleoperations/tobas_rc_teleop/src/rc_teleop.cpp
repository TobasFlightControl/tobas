#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/string.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/PoseTwistAccelCommand.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>
#include <tobas_msgs/GetArm.h>
#include <tobas_msgs/SetArm.h>

#include "../include/tobas_rc_teleop/rc_teleop.hpp"
#include "../include/tobas_rc_teleop/common.hpp"
#include "../include/tobas_rc_teleop/program_mode.hpp"
#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/position_yaw.hpp"
#include "../include/tobas_rc_teleop/rpy_thrust.hpp"
#include "../include/tobas_rc_teleop/pose_twist_accel.hpp"
#include "../include/tobas_rc_teleop/speed_roll_dpitch.hpp"

using namespace std;
using namespace ros::message_traits;
using namespace tobas_std;

namespace tobas_rc_teleop
{
RCTeleop::RCTeleop(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  // プログラムモードのダミーコントローラを設定
  controllers_[tobas::kFlightModeProgram] = make_unique<ProgramModeController>(drone_);

  // その他の飛行モードのコントローラを設定
  for (size_t i = 1; i < tobas::kNumFlightModes; ++i)
  {
    if (modes_[i] == split(DataType<tobas_msgs::PosVelAccYaw>::value(), '/').back())
      controllers_[i] = make_unique<PosVelAccYawController>(drone_);
    else if (modes_[i] == split(DataType<tobas_msgs::PositionYaw>::value(), '/').back())
      controllers_[i] = make_unique<PositionYawController>(drone_);
    else if (modes_[i] == split(DataType<tobas_msgs::RollPitchYawThrust>::value(), '/').back())
      controllers_[i] = make_unique<RollPitchYawThrustController>(drone_);
    else if (modes_[i] == split(DataType<tobas_msgs::PoseTwistAccelCommand>::value(), '/').back())
      controllers_[i] = make_unique<PoseTwistAccelController>(drone_);
    else if (modes_[i] == split(DataType<tobas_msgs::SpeedRollDeltaPitch>::value(), '/').back())
      controllers_[i] = make_unique<SpeedRollDeltaPitchController>(drone_);
    else
      exit("Invalid flight mode: " + modes_[i]);

    controllers_[i]->initialize(nh_, pnh_);
  }

  registerPublishers();
  registerSubscribers();

  get_arm_sc_ = nh_.serviceClient<tobas_msgs::GetArm>(tobas::kGetArmSrv);
  set_arm_sc_ = nh_.serviceClient<tobas_msgs::SetArm>(tobas::kSetArmSrv);
}

void RCTeleop::getRosParams()
{
  tobas_ros::getParam(pnh_, "stabilize_mode", modes_[tobas::kFlightModeStabilize]);
  tobas_ros::getParam(pnh_, "acrobat_mode", modes_[tobas::kFlightModeAcrobat]);
}

void RCTeleop::registerPublishers()
{
}

void RCTeleop::registerSubscribers()
{
  odom_sub_ = nh_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
  rcin_sub_ = nh_.subscribe(tobas::kRcInputTopic, 1, &self::rcInputCb, this, tcpNoDelay());
}

bool RCTeleop::isRotorsArmed()
{
  if (!get_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    error("Failed to connect to '", tobas::kGetArmSrv, "' service server.");
    return false;
  }

  tobas_msgs::GetArm get_arm_msg;
  if (!get_arm_sc_.call(get_arm_msg))
  {
    error("Failed to get arming state.");
    return false;
  }

  return get_arm_msg.response.arming;
}

bool RCTeleop::requestArmingRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    error("Failed to connect to '", tobas::kSetArmSrv, "' service server.");
    return false;
  }

  tobas_msgs::SetArm set_arm_msg;
  set_arm_msg.request.arming = true;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    error("Failed to arm rotors.");
    return false;
  }

  return true;
}

bool RCTeleop::requestDisarmingRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    error("Failed to connect to '", tobas::kSetArmSrv, "' service server.");
    return false;
  }

  tobas_msgs::SetArm set_arm_msg;
  set_arm_msg.request.arming = false;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    error("Failed to disarm rotors.");
    return false;
  }

  return true;
}

void RCTeleop::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
  if (odom->status != tobas_msgs::Odometry::NO_ERROR)
    return;

  odom_ = odom;
}

void RCTeleop::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void RCTeleop::rcInputCb(const tobas_msgs::RCInputConstPtr& rcin)
{
  if (rcin->error.error != tobas_msgs::RCInputError::E_NO_ERROR)
    return;

  switch (stage_)
  {
    case CHECK_PREREQUISITES:
    {
      if (odom_ != nullptr && battery_ != nullptr)
        stage_ = WAIT_FOR_ESTOP;
      break;
    }

    case WAIT_FOR_ESTOP:
    {
      if (rcin->e_stop)
      {
        info("RC transmitter is ready. "
             "To start control, set the E-Stop toggle OFF "
             "with the throttle lever lowered to the bottom.");
        stage_ = ESTOP_ON;
      }
      else
      {
        infoThrottle(kInfoPeriod, "Please start with the transmitter's E-Stop toggle ON.");
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
      if (rcin->thrust > kInitThrustThreshold)
      {
        warnThrottle(
          kWarnPeriod, "Please lower the throttle lever to the bottom before turning off E-Stop.");
        break;
      }
      if (!requestArmingRotors())
        break;

      // 問題なければコマンドを送信開始
      stage_ = FIRST_COMMAND;
      break;
    }

    case FIRST_COMMAND:
    {
      const auto& cur_mode = rcin->mode;
      if (cur_mode >= tobas::kNumFlightModes)
      {
        errorThrottle(kErrorPeriod, "Invalid flight mode.");
        return;
      }

      controllers_[cur_mode]->reset(*odom_);
      last_mode_ = cur_mode;
      info("First flight mode is set to \"", mode2str_.at(cur_mode), "\".");

      stage_ = RUNNING;
      break;
    }

    case RUNNING:
    {
      // E-Stopがオンになったら非常停止
      if (rcin->e_stop)
      {
        warn("Stopping rotors.");
        requestDisarmingRotors();
        stage_ = ESTOP_ON;
      }

      const auto& cur_mode = rcin->mode;
      if (cur_mode >= tobas::kNumFlightModes)
      {
        errorThrottle(kErrorPeriod, "Invalid flight mode.");
        return;
      }

      if (cur_mode != last_mode_)
      {
        last_mode_ = cur_mode;
        controllers_[cur_mode]->reset(*odom_);
        info("Flight mode changed to \"", mode2str_.at(cur_mode), "\".");
        break;
      }

      controllers_[cur_mode]->update(*rcin, *odom_, battery_->voltage);
      break;
    }

    default:
    {
      error("Invalid stage: ", static_cast<int>(stage_));
      break;
    }
  }
}
}  // namespace tobas_rc_teleop
