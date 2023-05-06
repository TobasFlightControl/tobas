#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_keyboard_teleop/keyboard_teleop.hpp"

#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44

#define OVER_SAMPLING 10.
#define INFO_PERIOD 1.
#define INSTRUCTION_PERIOD 10.

using namespace std;

CommandHandler::CommandHandler() : super()
{
  instruction_ = "Control your drone!\n"
                 "---------------------------\n"
                 "W/S       : increase/decrease linear velocity along x-axis in WCSs\n"
                 "A/D       : increase/decrease linear velocity along y-axis in WCSs\n"
                 "Left/Right: increase/decrease angular velocity along z-axis in WCSs\n"
                 "Ctrl-C    : quit\n";

  getRosParams();

  update_rate_ = key_repeat_freq_ * OVER_SAMPLING;  // 全ての入力を拾うためにオーバーサンプリング
  delta_pos_ = max_linvel_ / key_repeat_freq_;
  delta_rot_ = max_angvel_ / key_repeat_freq_;

  // z座標の初期値を制限の下限に設定
  cmd_.position.z = z_limit_.lower;

  prepare(0);

  registerPublishers();
  registerSubscribers();
  createTimers();
}

CommandHandler::~CommandHandler()
{
  tcsetattr(0, TCSANOW, &tempcopy_);
}

void CommandHandler::run()
{
  instruction_timer_.start();
  dh_ros::rosInfo(instruction_);

  char c = 0;
  ros::Rate rate(update_rate_);

  while (ros::ok())
  {
    if (read(0, &c, 1) < 0)
    {
      dh_ros::rosError("Failed to read keyboard input.");
      break;
    }

    // いきなりCtrl+Cを押すとループから抜けられずにバグるため，必ず"q"で抜けるようにする．
    // と思ったが普通にCtrl+Cで落ちてくれるっぽい．
    // if (c == 'q')
    // {
    //   dh_ros::rosInfo("q is detected. Command handling is terminated.");
    //   break;
    // }

    switch (c)
    {
      case 'w':  // X+
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving forward");
        cmd_.position.x = x_limit_.clamp(cmd_.position.x + delta_pos_);
        break;
      }
      case 's':  // X-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving backward");
        cmd_.position.x = x_limit_.clamp(cmd_.position.x - delta_pos_);
        break;
      }
      case 'a':  // Y+
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving left");
        cmd_.position.y = y_limit_.clamp(cmd_.position.y + delta_pos_);
        break;
      }
      case 'd':  // Y-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving right");
        cmd_.position.y = y_limit_.clamp(cmd_.position.y - delta_pos_);
        break;
      }
      case KEYCODE_U:  // Z+
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving up");
        cmd_.position.z = z_limit_.clamp(cmd_.position.z + delta_pos_);
        break;
      }
      case KEYCODE_D:  // Z-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving down");
        cmd_.position.z = z_limit_.clamp(cmd_.position.z - delta_pos_);
        break;
      }
      case KEYCODE_L:  // Yaw+
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Rotating left");
        cmd_.yaw = yaw_limit_.clamp(cmd_.yaw + delta_rot_);
        break;
      }
      case KEYCODE_R:  // Yaw-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Rotating right");
        cmd_.yaw = yaw_limit_.clamp(cmd_.yaw - delta_rot_);
        break;
      }
    }

    // キーボード入力をリセット
    // リセットしないと同じコマンドが連続して入力されてしまう．
    c = 0;

    cmd_pub_.publish(cmd_);

    ros::spinOnce();
    rate.sleep();
  }
}

void CommandHandler::getRosParams()
{
  dh_ros::getParam("~key_repeat_freq", key_repeat_freq_);
  dh_ros::getParam("~max_linear_velocity", max_linvel_);
  dh_ros::getParam("~max_angular_velocity", max_angvel_);
  dh_ros::getParam("~pose_limit/x/min", x_limit_.lower);
  dh_ros::getParam("~pose_limit/x/max", x_limit_.upper);
  dh_ros::getParam("~pose_limit/y/min", y_limit_.lower);
  dh_ros::getParam("~pose_limit/y/max", y_limit_.upper);
  dh_ros::getParam("~pose_limit/z/min", z_limit_.lower);
  dh_ros::getParam("~pose_limit/z/max", z_limit_.upper);
  dh_ros::getParam("~pose_limit/yaw/min", yaw_limit_.lower);
  dh_ros::getParam("~pose_limit/yaw/max", yaw_limit_.upper);

  ROS_ASSERT(key_repeat_freq_ > 0.);
  ROS_ASSERT(max_linvel_ > 0.);
  ROS_ASSERT(max_angvel_ > 0.);
  ROS_ASSERT(x_limit_.lower < 0. && 0. < x_limit_.upper);
  ROS_ASSERT(y_limit_.lower < 0. && 0. < y_limit_.upper);
  ROS_ASSERT(0. < z_limit_.lower && z_limit_.lower < z_limit_.upper);
  ROS_ASSERT(yaw_limit_.lower < 0. && 0. < yaw_limit_.upper);
}

void CommandHandler::registerPublishers()
{
  cmd_pub_ = nh_.advertise<CmdMsg>("command/position_yaw", 1, false);
}

void CommandHandler::registerSubscribers()
{
}

void CommandHandler::createTimers()
{
  instruction_timer_ = nh_.createTimer(
    ros::Duration(INSTRUCTION_PERIOD), &CommandHandler::instructionTimerCb, this, false, false);
}

void CommandHandler::prepare(int fd)
{
  tcgetattr(fd, &tempcopy_);
  memcpy(&changed_, &tempcopy_, sizeof(termios));

  changed_.c_lflag &= ~(ICANON | ECHO);
  changed_.c_cc[VEOL] = 1;
  changed_.c_cc[VEOF] = 2;

  // 入力受付のタイムリミットを設定
  // https://stackoverflow.com/questions/2917881/how-to-implement-a-timeout-in-read-function-call
  changed_.c_cc[VMIN] = 0.;
  changed_.c_cc[VTIME] = 10. / update_rate_;

  tcsetattr(fd, TCSANOW, &changed_);
}

void CommandHandler::checkTopicsTimerCb(const ros::TimerEvent& event)
{
}

void CommandHandler::instructionTimerCb(const ros::TimerEvent&)
{
  dh_ros::rosInfo(instruction_);
}
