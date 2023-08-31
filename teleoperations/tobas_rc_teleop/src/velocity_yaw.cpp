#include <dh_ros_tools/rosparam.hpp>

#include "../include/tobas_rc_teleop/velocity_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_rc_teleop
{
RcinToVelocityYaw::RcinToVelocityYaw(ros::NodeHandle nh, ros::NodeHandle pnh)
  : super(nh, pnh), state_(CHECK_PREREQUISITES), bs_received_(false)
{
  getRosParams();

  dead_zone_.lower = -dead_zone_rate_ / 2;
  dead_zone_.upper = dead_zone_rate_ / 2;

  vel_yaw_.level.data = tobas_msgs::CommandLevel::MANUAL;
  vel_yaw_.frame_id.data = tobas_msgs::FrameId::GLOBAL;

  registerPublishers();
  registerSubscribers();
}

void RcinToVelocityYaw::getRosParams()
{
  dh_ros::getParam(
    pnh_, "max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorizontalVelocity, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh_, "max_vertical_velocity", max_ver_vel_, kDefaultMaxVerticalVelocity, dh_ros::POSITIVE);
  dh_ros::getParam(pnh_, "max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);

  dh_ros::getParam(
    pnh_, "dead_zone_rate", dead_zone_rate_, kDefaultDeadZoneRate, dh_ros::NON_NEGATIVE);
  if (dead_zone_rate_ >= 1.)
  {
    rosthrow("'dead_zone_rate' must be lower than 1.");
  }
}

void RcinToVelocityYaw::registerPublishers()
{
  vel_yaw_pub_ = nh_.advertise<tobas_msgs::VelocityYaw>("command/velocity_yaw", 1);
  event_pub_ = nh_.advertise<tobas_msgs::Event>("event", 1);
}

void RcinToVelocityYaw::registerSubscribers()
{
  bs_sub_ = nh_.subscribe("base_state", 1, &RcinToVelocityYaw::baseStateCb, this);
  rcin_sub_ = nh_.subscribe("rc_input", 1, &RcinToVelocityYaw::rcInputCb, this);
}

void RcinToVelocityYaw::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}

void RcinToVelocityYaw::baseStateCb(const tobas_msgs::BaseState& bs)
{
  if (!bs_received_)
  {
    bs_received_ = true;
  }

  bs_ = bs;
}

void RcinToVelocityYaw::rcInputCb(const tobas_msgs::RCInput& rcin)
{
  switch (state_)
  {
    case CHECK_PREREQUISITES:
    {
      if (bs_received_)
      {
        state_ = FIRST_RCIN;
      }
      break;
    }

    case FIRST_RCIN:
    {
      if (rcin.toggle)
      {
        rosErrorThrottle(
          kErrorPeriod, "Please start with the transmitter's toggle in the OFF position.");
      }
      else
      {
        rosInfo("RC transmitter is ready. Toggle on to start control.");
        state_ = TOGGLE_OFF;
      }
      break;
    }

    case TOGGLE_OFF:
    {
      if (rcin.toggle)
      {
        t_last_rcin_ = ros::Time::now();
        vel_yaw_.yaw = bs_.pose.euler.yaw;  // 最初は現在のヨー角を指令
        state_ = TOGGLE_ON;
      }
      break;
    }

    case TOGGLE_ON:
    {
      if (!rcin.toggle)
      {
        rosInfo("The toggle has changed from ON to OFF. Shutting down the system.");
        requestShutdown();
        ros::shutdown();
      }

      // 並進速度を更新
      vel_yaw_.vel.x(
        dead_zone_.inRange(rcin.pitch) ? 0. :
                                         remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_));
      vel_yaw_.vel.y(
        dead_zone_.inRange(rcin.roll) ? 0. :
                                        -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_));
      vel_yaw_.vel.z(remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_));

      // Yawの目標値を更新
      const ros::Time now = ros::Time::now();
      const auto dt = (now - t_last_rcin_).toSec();
      t_last_rcin_ = now;
      const auto yawrate =
        dead_zone_.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);
      vel_yaw_.yaw += yawrate * dt;

      // コマンドを発行
      vel_yaw_pub_.publish(vel_yaw_);

      break;
    }

    default:
    {
      rosthrow("Invalid state: " << state_);
    }
  }
}
}  // namespace tobas_rc_teleop
