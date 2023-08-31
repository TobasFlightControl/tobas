#pragma once

#include <dh_std_tools/range.hpp>

#include <tobas_tools/node.hpp>

#include "../../include/tobas_keyboard_teleop/x11.hpp"
#include "../../include/tobas_keyboard_teleop/keyboard_reader.hpp"

namespace tobas_keyboard_teleop
{
/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class PositionYawPublisher : public tobas::BaseNode
{
  static constexpr double kDefaultMaxLinearVelocity = 3.;       // [m/s]
  static constexpr double kDefaultMaxAngularVelocity = M_PI_2;  // [rad]
  static constexpr double kDefaultMinimumX = -10.;              // [m]
  static constexpr double kDefaultMaximumX = +10.;              // [m]
  static constexpr double kDefaultMinimumY = -10.;              // [m]
  static constexpr double kDefaultMaximumY = +10.;              // [m]
  static constexpr double kDefaultMinimumZ = -10.;              // [m]
  static constexpr double kDefaultMaximumZ = +10.;              // [m]
  static constexpr double kDefaultMinimumYaw = -M_PI;           // [rad]
  static constexpr double kDefaultMaximumYaw = M_PI;            // [rad]

  using super = tobas::BaseNode;

public:
  explicit PositionYawPublisher(ros::NodeHandle nh, ros::NodeHandle pnh);

  void run();

private:
  const XkbControlsPtr keyboard_;
  KeyboardReader key_reader_;

  // 固定値
  std::string instruction_;
  double delta_pos_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;  // 1度のキーボード入力での回転位置の変化量

  // rosparams
  double max_linvel_;  // 並進速度の大きさの最大値
  double max_angvel_;  // 回転速度の大きさの最大値
  dh_std::Range<double> x_limit_;
  dh_std::Range<double> y_limit_;
  dh_std::Range<double> z_limit_;
  dh_std::Range<double> yaw_limit_;

  // PubSub
  ros::Publisher cmd_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::Event& event) override;
};
}  // namespace tobas_keyboard_teleop
