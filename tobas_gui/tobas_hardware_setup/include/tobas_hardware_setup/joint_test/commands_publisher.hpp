#pragma once

#include <QWidget>
#include <QPushButton>
#include <QTimer>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>

namespace gui
{
namespace hardware_setup
{
class JointCommandsPublisherWidget : public QWidget
{
  Q_OBJECT

  using self = JointCommandsPublisherWidget;
  using super = QWidget;

  static constexpr int kChannelSize = 8;  // TODO: ハードウェアの最大チャンネル数に合わせる
  static constexpr int kMaxRows = kChannelSize / 2;
  static constexpr int kPublishPeriod = 10;           // [ms]
  static constexpr double kDefaultMaxVel = 2 * M_PI;  // [rad/s]
  static constexpr double kDefaultMaxEff = 10.;       // [Nm]

public:
  explicit JointCommandsPublisherWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone);

  void updateInternalDataStructures();

  void start();
  void stop();

private:
  const rclcpp::Node::SharedPtr node_;
  const kdl::Tree& tree_;
  const tobas::Drone& drone_;

  kdl::TreeJointParser joint_parser_;

  std::array<std::string, kChannelSize> jnt_names_;
  std::array<tobas::jnt_cmd_iface_t, kChannelSize> cmd_iface_;
  std::array<double, kChannelSize> home_pos_;
  std::array<qt::DoubleSliderDisplay*, kChannelSize> commanders_;

  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> pos_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> vel_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> eff_pub_;

  QTimer publish_timer_;

  void publishCurrentValues();

  void publishTimerCb();

private Q_SLOTS:
  void onValueChanged();
};
}  // namespace hardware_setup
}  // namespace gui
