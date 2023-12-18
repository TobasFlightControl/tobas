#pragma once

#include <dh_std_tools/range.hpp>
#include <dh_std_tools/keyboard_reader.hpp>
#include <dh_kdl/frames.hpp>

#include <tobas_tools/node.hpp>

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

  using self = PositionYawPublisher;
  using super = tobas::BaseNode;

public:
  explicit PositionYawPublisher(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

  void run();

private:
  dh_std::KeyboardReader key_reader_;

  KDL::Vector cmd_pos_;
  double cmd_yaw_;

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

  // Publishers
  ros::Publisher pos_yaw_pub_;
  ros::Publisher pvay_pub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
};
}  // namespace tobas_keyboard_teleop
