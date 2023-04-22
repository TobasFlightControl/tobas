#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <dh_std_tools/math.hpp>
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

CommandHandler::CommandHandler()
{
  instruction_ = "Control your drone!\n"
                 "---------------------------\n"
                 "W/S       : increase/decrease linear velocity along x-axis in WCSs\n"
                 "A/D       : increase/decrease linear velocity along y-axis in WCSs\n"
                 "Left/Right: increase/decrease angular velocity along z-axis in WCSs\n"
                 "Ctrl-C    : quit\n";

  getParams();

  update_rate_ = key_repeat_freq_ * OVER_SAMPLING;  // 全ての入力を拾うためにオーバーサンプリング
  delta_pos_ = max_linvel_ / key_repeat_freq_;
  delta_rot_ = max_angvel_ / key_repeat_freq_;

  // コマンドをGlobal Positionモードに設定
  cmd_.mode = CmdMsg::GLOBAL_POSITION;

  // z座標の初期値を制限の下限に設定
  cmd_.target_position.z = z_limit_.lower;

  prepare(0);

  cmd_pub_ = nh_.advertise<CmdMsg>("/tobas_controller/command", 1, false);

  instruction_timer_ = nh_.createTimer(
    ros::Duration(INSTRUCTION_PERIOD), &CommandHandler::instructionTimerCb, this, false, false);
}

CommandHandler::~CommandHandler()
{
  tcsetattr(0, TCSANOW, &tempcopy_);
  instruction_timer_.stop();
}

void CommandHandler::run()
{
  auto& x = cmd_.target_position.x;
  auto& y = cmd_.target_position.y;
  auto& z = cmd_.target_position.z;
  auto& yaw = cmd_.target_yaw_angle;

  char c = 0;
  ros::Rate rate(update_rate_);

  // spin & sleepの後にタイマーを起動することで，タイマーが複数回呼ばれることを防ぐ
  ros::spinOnce();
  ros::Duration(0.1).sleep();
  instruction_timer_.start();
  dh_ros::rosInfo(instruction_);

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
        x = dh_std::clamp(x + delta_pos_, x_limit_.lower, x_limit_.upper, "X");
        break;
      }
      case 's':  // X-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving backward");
        x = dh_std::clamp(x - delta_pos_, x_limit_.lower, x_limit_.upper, "X");
        break;
      }
      case 'a':  // Y+
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving left");
        y = dh_std::clamp(y + delta_pos_, y_limit_.lower, y_limit_.upper, "Y");
        break;
      }
      case 'd':  // Y-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving right");
        y = dh_std::clamp(y - delta_pos_, y_limit_.lower, y_limit_.upper, "Y");
        break;
      }
      case KEYCODE_U:  // Z+
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving up");
        z = dh_std::clamp(z + delta_pos_, z_limit_.lower, z_limit_.upper, "Z");
        break;
      }
      case KEYCODE_D:  // Z-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Moving down");
        z = dh_std::clamp(z - delta_pos_, z_limit_.lower, z_limit_.upper, "Z");
        break;
      }
      case KEYCODE_L:  // yaw+
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Rotating left");
        yaw = dh_std::clamp(yaw + delta_rot_, yaw_limit_.lower, yaw_limit_.upper, "Yaw");
        break;
      }
      case KEYCODE_R:  // yaw-
      {
        dh_ros::rosInfoThrottle(INFO_PERIOD, "Rotating right");
        yaw = dh_std::clamp(yaw - delta_rot_, yaw_limit_.lower, yaw_limit_.upper, "Yaw");
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

void CommandHandler::getParams()
{
  key_repeat_freq_ = dh_ros::getParam<double>("~key_repeat_freq");
  max_linvel_ = dh_ros::getParam<double>("~max_linear_velocity");
  max_angvel_ = dh_ros::getParam<double>("~max_angular_velocity");
  x_limit_.lower = dh_ros::getParam<double>("~pose_limit/x/min");
  x_limit_.upper = dh_ros::getParam<double>("~pose_limit/x/max");
  y_limit_.lower = dh_ros::getParam<double>("~pose_limit/y/min");
  y_limit_.upper = dh_ros::getParam<double>("~pose_limit/y/max");
  z_limit_.lower = dh_ros::getParam<double>("~pose_limit/z/min");
  z_limit_.upper = dh_ros::getParam<double>("~pose_limit/z/max");
  yaw_limit_.lower = dh_ros::getParam<double>("~pose_limit/yaw/min");
  yaw_limit_.upper = dh_ros::getParam<double>("~pose_limit/yaw/max");

  ROS_ASSERT(key_repeat_freq_ > 0.);
  ROS_ASSERT(max_linvel_ > 0.);
  ROS_ASSERT(max_angvel_ > 0.);
  ROS_ASSERT(x_limit_.lower < 0. && 0. < x_limit_.upper);
  ROS_ASSERT(y_limit_.lower < 0. && 0. < y_limit_.upper);
  ROS_ASSERT(0. < z_limit_.lower && z_limit_.lower < z_limit_.upper);
  ROS_ASSERT(yaw_limit_.lower < 0. && 0. < yaw_limit_.upper);
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

void CommandHandler::instructionTimerCb(const ros::TimerEvent&)
{
  dh_ros::rosInfo(instruction_);
}
