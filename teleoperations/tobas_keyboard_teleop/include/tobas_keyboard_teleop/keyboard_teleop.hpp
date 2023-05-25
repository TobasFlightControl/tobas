#pragma once

#include <ros/ros.h>

#include <dh_std_tools/range.hpp>
#include <dh_ros_tools/node.hpp>

#include <tobas_msgs/PositionYaw.h>

#include "../../include/tobas_keyboard_teleop/x11.hpp"
#include "../../include/tobas_keyboard_teleop/keyboard_reader.hpp"

namespace tobas_keyboard_teleop
{
/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class CommandHandler : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using CmdMsg = tobas_msgs::PositionYaw;

public:
  explicit CommandHandler();

  void run();

private:
  const XkbControlsPtr keyboard_;
  KeyboardReader key_reader_;

  std::string instruction_;
  double update_rate_;
  double delta_pos_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;  // 1度のキーボード入力での回転位置の変化量
  CmdMsg cmd_;

  // rosparams
  double max_linvel_;  // 並進速度の大きさの最大値
  double max_angvel_;  // 回転速度の大きさの最大値
  dh_std::Range<double> x_limit_;
  dh_std::Range<double> y_limit_;
  dh_std::Range<double> z_limit_;
  dh_std::Range<double> yaw_limit_;

  // PubSub
  ros::Publisher cmd_pub_;

  dh_ros::Timer instruction_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
  void instructionTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_keyboard_teleop
