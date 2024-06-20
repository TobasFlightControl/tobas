#pragma once

#include <tobas_std_tools/range.hpp>
#include <tobas_keyboard/keyboard_reader.hpp>
#include <tobas_kdl/frames.hpp>

#include <tobas_tools/node.hpp>

namespace tobas_keyboard_teleop
{
/**
 * @brief キーボード入力を受け取り，コマンドを発行する．
 */
class PositionYawPublisher : public tobas::BaseNode
{
  static constexpr double kDefaultMaxLinearVelocity = 3.;       // [m/s]
  static constexpr double kDefaultMaxAngularVelocity = M_PI_2;  // [rad/s]
  static constexpr double kDefaultMinimumX = -10.;              // [m]
  static constexpr double kDefaultMaximumX = +10.;              // [m]
  static constexpr double kDefaultMinimumY = -10.;              // [m]
  static constexpr double kDefaultMaximumY = +10.;              // [m]
  static constexpr double kDefaultMinimumZ = -10.;              // [m]
  static constexpr double kDefaultMaximumZ = +10.;              // [m]
  static constexpr double kDefaultMinimumYaw = -M_PI;           // [rad]
  static constexpr double kDefaultMaximumYaw = M_PI;            // [rad]

  using self = PositionYawPublisher;
  using super = tobas::BaseNode;

public:
  explicit PositionYawPublisher(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

  void run();

private:
  keyboard::KeyboardReader key_reader_;

  tobas_kdl::Vector cmd_pos_;
  double cmd_yaw_;

  // 固定値
  std::string instruction_;
  double delta_pos_;  // 1度のキーボード入力での並進位置の変化量
  double delta_rot_;  // 1度のキーボード入力での回転位置の変化量

  // rosparams
  double max_linvel_;  // 並進速度の大きさの最大値
  double max_angvel_;  // 回転速度の大きさの最大値
  tobas_std::Range<double> x_limit_;
  tobas_std::Range<double> y_limit_;
  tobas_std::Range<double> z_limit_;
  tobas_std::Range<double> yaw_limit_;

  // Publishers
  ros::Publisher pos_yaw_pub_;
  ros::Publisher pvay_pub_;

  void getRosParams();
};
}  // namespace tobas_keyboard_teleop
