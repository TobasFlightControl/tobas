#pragma once

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <tobas_std_tools/range.hpp>
#include <tobas_std_tools/keyboard_reader.hpp>
#include <tobas_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/trim_conditions.hpp>
#include <tobas_msgs/SpeedRollDeltaPitch.h>

namespace tobas_keyboard_teleop
{
/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class SpeedRollDeltaPitchPublisher : public tobas::BaseNode
{
  static constexpr double kDefaultMaxLinearAcceleration = 2.;
  static constexpr double kDefaultMaxAngularVelocity = M_PI_2;
  static constexpr double kDefaultMaximumRoll = M_PI_2;
  static constexpr double kDefaultMaximumDeltaPitch = M_PI_4;

  using self = SpeedRollDeltaPitchPublisher;
  using super = tobas::BaseNode;

public:
  explicit SpeedRollDeltaPitchPublisher(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

  void run();

private:
  tobas::Drone drone_;
  tobas::TrimConditions trim_;
  tobas_std::KeyboardReader key_reader_;

  // 固定値
  std::string instruction_;
  KDL::JntArray q_0_;
  double delta_speed_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;    // 1度のキーボード入力での回転位置の変化量

  // 可変値
  bool is_initialized_ = false;
  bool pressure_received_ = false;
  double air_density_;  // 現在の大気密度
  tobas_msgs::SpeedRollDeltaPitch cmd_;

  // rosparams
  double max_linacc_;       // 並進加速度の大きさの最大値
  double max_angvel_;       // 回転速度の大きさの最大値
  double max_roll_;         // ロール角の最大値
  double max_delta_pitch_;  // ピッチ角の釣り合いからの偏差の最大値

  // PubSub
  ros::Publisher cmd_pub_;
  ros::Subscriber air_pressure_sub_;

  // Timer
  tobas_ros::Timer check_topics_timer_;
  tobas_ros::Timer instruction_timer_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void airPressureCb(const sensor_msgs::FluidPressureConstPtr& msg);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void instructionTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_keyboard_teleop
