#pragma once

#include <random>

#include <QTimer>
#include <QVBoxLayout>

#include <tobas_qt_tools/widgets/slider_display.hpp>
#include <tobas_ros2_tools/definitions.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_visualization_msgs/msg/display_robot_state.hpp>

#include "./robot_info.hpp"

namespace gui
{
namespace sa
{
class JointStatePublisherWidget : public QWidget
{
  Q_OBJECT

  using self = JointStatePublisherWidget;
  using super = QWidget;

  static constexpr int kButtonHeight = 40;

public:
  explicit JointStatePublisherWidget(rclcpp::Node::SharedPtr node, const RobotInfo& robot);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const RobotInfo& robot_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  QVBoxLayout* slider_rows_;

  sensor_msgs::msg::JointState js_;
  std::vector<qt::DoubleSliderDisplay*> sliders_;

  ros2::PublisherPtr<sensor_msgs::msg::JointState> js_pub_;
  ros2::PublisherPtr<tobas_visualization_msgs::msg::DisplayRobotState> drs_pub_;

  QTimer publish_timer_;

  void publish();

private Q_SLOTS:
  void onValueChanged(double value, const std::string& jnt_name);
  void onCenterButtonClicked();
  void onRandomButtonClicked();
};
}  // namespace sa
}  // namespace gui
