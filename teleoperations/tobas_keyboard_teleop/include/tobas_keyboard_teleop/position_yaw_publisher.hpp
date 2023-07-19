#pragma once

#include <ros/ros.h>

#include <dh_std_tools/range.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PositionYaw.h>

#include "../../include/tobas_keyboard_teleop/x11.hpp"
#include "../../include/tobas_keyboard_teleop/keyboard_reader.hpp"

namespace tobas_keyboard_teleop
{
static constexpr double kInitialX = 0.;
static constexpr double kInitialY = 0.;
static constexpr double kInitialZ = 1.;
static constexpr double kInitialYaw = 0.;

static constexpr double kDefaultMaxLinearVelocity = 5.;
static constexpr double kDefaultMaxAngularVelocity = M_PI_2;
static constexpr double kDefaultMinimumX = -10.;
static constexpr double kDefaultMaximumX = +10.;
static constexpr double kDefaultMinimumY = -10.;
static constexpr double kDefaultMaximumY = +10.;
static constexpr double kDefaultMinimumZ = -10.;
static constexpr double kDefaultMaximumZ = +10.;
static constexpr double kDefaultMinimumYaw = -M_PI;
static constexpr double kDefaultMaximumYaw = M_PI;

/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class PositionYawPublisher : public tobas::BaseNode
{
  using super = tobas::BaseNode;

public:
  explicit PositionYawPublisher();

  void run();

private:
  const XkbControlsPtr keyboard_;
  KeyboardReader key_reader_;

  // 固定値
  std::string instruction_;
  double delta_pos_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;  // 1度のキーボード入力での回転位置の変化量

  // 可変値
  tobas_msgs::PositionYaw cmd_;

  // rosparams
  double max_linvel_;  // 並進速度の大きさの最大値
  double max_angvel_;  // 回転速度の大きさの最大値
  dh_std::Range<double> x_limit_;
  dh_std::Range<double> y_limit_;
  dh_std::Range<double> z_limit_;
  dh_std::Range<double> yaw_limit_;

  // PubSub
  ros::Publisher cmd_pub_;

  // Timer
  dh_ros::Timer instruction_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
  void instructionTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_keyboard_teleop
