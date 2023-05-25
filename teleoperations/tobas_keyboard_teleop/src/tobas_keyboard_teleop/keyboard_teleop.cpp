#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_keyboard_teleop/keyboard_teleop.hpp"
#include "../../include/tobas_keyboard_teleop/constants.hpp"

using namespace std;

namespace tobas_keyboard_teleop
{
CommandHandler::CommandHandler() : super(), keyboard_(getKeyboardControls())
{
  instruction_ = "Control your drone!\n"
                 "---------------------------\n"
                 "W/S       : increase/decrease linear velocity along x-axis in WCSs\n"
                 "A/D       : increase/decrease linear velocity along y-axis in WCSs\n"
                 "Left/Right: increase/decrease angular velocity along z-axis in WCSs\n"
                 "Ctrl-C    : quit\n";

  getRosParams();

  const auto repeat_interval = keyboard_->repeat_interval * 1e-3;  // ms -> s
  rosInfo("Keyboard repeat interval is " << keyboard_->repeat_interval << " [ms].");

  delta_pos_ = max_linvel_ * repeat_interval;
  delta_rot_ = max_angvel_ * repeat_interval;

  // z座標の初期値を制限の下限に設定
  cmd_.pos.z(z_limit_.lower);

  registerPublishers();
  registerSubscribers();
  createTimers();
}

void CommandHandler::run()
{
  instruction_timer_.start();
  rosInfo(instruction_);

  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    const auto c = key_reader_.readKey();

    switch (c)
    {
      case kKeyCode_W:  // X+
      {
        rosInfoThrottle(kInfoPeriod, "Moving forward");
        cmd_.pos.x(x_limit_.clamp(cmd_.pos.x() + delta_pos_));
        break;
      }
      case kKeyCode_S:  // X-
      {
        rosInfoThrottle(kInfoPeriod, "Moving backward");
        cmd_.pos.x(x_limit_.clamp(cmd_.pos.x() - delta_pos_));
        break;
      }
      case kKeyCode_A:  // Y+
      {
        rosInfoThrottle(kInfoPeriod, "Moving left");
        cmd_.pos.y(y_limit_.clamp(cmd_.pos.y() + delta_pos_));
        break;
      }
      case kKeyCode_D:  // Y-
      {
        rosInfoThrottle(kInfoPeriod, "Moving right");
        cmd_.pos.y(y_limit_.clamp(cmd_.pos.y() - delta_pos_));
        break;
      }
      case kKeyCode_Up:  // Z+
      {
        rosInfoThrottle(kInfoPeriod, "Moving up");
        cmd_.pos.z(z_limit_.clamp(cmd_.pos.z() + delta_pos_));
        break;
      }
      case kKeyCode_Down:  // Z-
      {
        rosInfoThrottle(kInfoPeriod, "Moving down");
        cmd_.pos.z(z_limit_.clamp(cmd_.pos.z() - delta_pos_));
        break;
      }
      case kKeyCode_Left:  // Yaw+
      {
        rosInfoThrottle(kInfoPeriod, "Rotating left");
        cmd_.yaw = yaw_limit_.clamp(cmd_.yaw + delta_rot_);
        break;
      }
      case kKeyCode_Right:  // Yaw-
      {
        rosInfoThrottle(kInfoPeriod, "Rotating right");
        cmd_.yaw = yaw_limit_.clamp(cmd_.yaw - delta_rot_);
        break;
      }
    }

    cmd_pub_.publish(cmd_);

    ros::spinOnce();
    rate.sleep();
  }
}

void CommandHandler::getRosParams()
{
  dh_ros::getParam("~max_linear_velocity", max_linvel_, dh_ros::POSITIVE);
  dh_ros::getParam("~max_angular_velocity", max_angvel_, dh_ros::POSITIVE);
  dh_ros::getParam("~pose_limit/x/min", x_limit_.lower);
  dh_ros::getParam("~pose_limit/x/max", x_limit_.upper);
  dh_ros::getParam("~pose_limit/y/min", y_limit_.lower);
  dh_ros::getParam("~pose_limit/y/max", y_limit_.upper);
  dh_ros::getParam("~pose_limit/z/min", z_limit_.lower);
  dh_ros::getParam("~pose_limit/z/max", z_limit_.upper);
  dh_ros::getParam("~pose_limit/yaw/min", yaw_limit_.lower);
  dh_ros::getParam("~pose_limit/yaw/max", yaw_limit_.upper);

  ROS_ASSERT(x_limit_.isValid());
  ROS_ASSERT(y_limit_.isValid());
  ROS_ASSERT(z_limit_.isValid());
  ROS_ASSERT(yaw_limit_.isValid());
}

void CommandHandler::registerPublishers()
{
  cmd_pub_ = nh_.advertise<CmdMsg>("command/position_yaw", 1);
}

void CommandHandler::registerSubscribers()
{
}

void CommandHandler::createTimers()
{
  instruction_timer_ = nh_.createTimer(
    ros::Duration(kInstructionPeriod), &CommandHandler::instructionTimerCb, this, false, false);
}

void CommandHandler::checkTopicsTimerCb(const ros::TimerEvent& event)
{
}

void CommandHandler::instructionTimerCb(const ros::TimerEvent&)
{
  rosInfo(instruction_);
}
}  // namespace tobas_keyboard_teleop
