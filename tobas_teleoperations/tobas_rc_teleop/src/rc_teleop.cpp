#include <std_srvs/SetBool.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/string.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/PoseTwistAccelCommand.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>

#include "../include/tobas_rc_teleop/rc_teleop.hpp"
#include "../include/tobas_rc_teleop/common.hpp"
#include "../include/tobas_rc_teleop/program_mode.hpp"
#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/position_yaw.hpp"
#include "../include/tobas_rc_teleop/velocity_yaw.hpp"
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
    else if (modes_[i] == split(DataType<tobas_msgs::VelocityYaw>::value(), '/').back())
      controllers_[i] = make_unique<VelocityYawController>(drone_);
    else if (modes_[i] == split(DataType<tobas_msgs::RollPitchYawThrust>::value(), '/').back())
      controllers_[i] = make_unique<RollPitchYawThrustController>(drone_);
    else if (modes_[i] == split(DataType<tobas_msgs::PoseTwistAccelCommand>::value(), '/').back())
      controllers_[i] = make_unique<PoseTwistAccelController>(drone_);
    else if (modes_[i] == split(DataType<tobas_msgs::SpeedRollDeltaPitch>::value(), '/').back())
      controllers_[i] = make_unique<SpeedRollDeltaPitchController>(drone_);
    else
      ROS_EXIT_NAMED(nh_, name_, "Invalid flight mode: " + modes_[i]);

    controllers_[i]->initialize(nh_, pnh_);
  }

  registerPublishers();
  registerSubscribers();

  set_arm_sc_ = nh_.serviceClient<std_srvs::SetBool>(tobas::kSetArmSrv);
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

void RCTeleop::requestDisarmingRotors()
{
  if (!set_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    rosError(name_, "Failed to connect to '" << tobas::kSetArmSrv << "' service server.");
    return;
  }

  std_srvs::SetBool set_arm_msg;
  set_arm_msg.request.data = false;
  if (!set_arm_sc_.call(set_arm_msg) || !set_arm_msg.response.success)
  {
    rosError(name_, "Failed to disarm rotors.");
    return;
  }
}

void RCTeleop::odomCb(const tobas_msgs::OdometryConstPtr& odom)
{
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
        TOBAS_GOOD("RC transmitter is ready. Set E-Stop toggle OFF to start control.");
        stage_ = ESTOP_ON;
      }
      else
      {
        rosInfoThrottle(
          kInfoPeriod, name_, "Please start with the transmitter's E-Stop toggle ON.");
      }
      break;
    }

    case ESTOP_ON:
    {
      if (!rcin->e_stop)
        stage_ = FIRST_COMMAND;
      break;
    }

    case FIRST_COMMAND:
    {
      const auto& cur_mode = rcin->mode;
      if (cur_mode >= controllers_.size())
      {
        rosErrorThrottle(
          kErrorPeriod, name_,
          "You tried to set flight mode " << static_cast<int>(cur_mode)
                                          << ", which is out of range.");
        return;
      }

      controllers_[cur_mode]->reset(*odom_);
      last_mode_ = cur_mode;
      rosInfo(name_, "First flight mode is set to " << static_cast<int>(cur_mode));

      stage_ = RUNNING;
      break;
    }

    case RUNNING:
    {
      if (rcin->e_stop)
      {
        rosWarn(name_, "Stopping rotors.");
        requestDisarmingRotors();
        stage_ = DISARMED;
      }

      const auto& cur_mode = rcin->mode;
      if (cur_mode >= controllers_.size())
      {
        rosErrorThrottle(
          kErrorPeriod, name_,
          "You tried to set flight mode " << static_cast<int>(cur_mode)
                                          << ", which is out of range.");
        return;
      }

      if (cur_mode != last_mode_)
      {
        controllers_[cur_mode]->reset(*odom_);
        rosInfo(
          name_, "Flight mode changed from " << static_cast<int>(last_mode_) << " to "
                                             << static_cast<int>(cur_mode) << ".");
        last_mode_ = cur_mode;
        break;
      }

      controllers_[cur_mode]->update(*rcin, *odom_, battery_->voltage);

      break;
    }

    case DISARMED:
    {
      // TODO: tobas_msgs::EventのリセットでWAIT_FOR_ESTOPに戻る
      break;
    }

    default:
    {
      ROS_EXIT_NAMED(nh_, name_, "Invalid stage: " << static_cast<int>(stage_));
    }
  }
}
}  // namespace tobas_rc_teleop
