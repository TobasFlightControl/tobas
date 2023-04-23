#pragma once

#include <ros/ros.h>
#include <termios.h>

#include <dh_std_tools/struct.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_msgs/Command.h>

/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class CommandHandler
{
  using CmdMsg = tobas_msgs::Command;

public:
  explicit CommandHandler();
  ~CommandHandler();

  void run();

private:
  ros::NodeHandle nh_;

  // rosparam
  std::string drone_name_;
  double key_repeat_freq_;  // キーボードの連続入力の周波数(PC依存)
  double max_linvel_;       // 並進速度の大きさの最大値
  double max_angvel_;       // 回転速度の大きさの最大値
  dh_std::Range<double> x_limit_;
  dh_std::Range<double> y_limit_;
  dh_std::Range<double> z_limit_;
  dh_std::Range<double> yaw_limit_;

  // other
  std::string instruction_;
  termios tempcopy_, changed_;
  double update_rate_;
  double delta_pos_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;  // 1度のキーボード入力での回転位置の変化量
  CmdMsg cmd_;

  ros::Publisher cmd_pub_;
  dh_ros::Timer instruction_timer_;

  void getRosParams();
  void prepare(int fd);
  void instructionTimerCb(const ros::TimerEvent&);
};
