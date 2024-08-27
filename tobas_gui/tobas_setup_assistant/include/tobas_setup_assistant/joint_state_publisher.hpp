#pragma once

#include <QVBoxLayout>

#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>

#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_qt_tools/widgets/widget.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>

namespace gui
{
namespace setup_assistant
{
class SetupAssistant;

class JointStatePublisherWidget : public qt::Widget
{
  Q_OBJECT

  using self = JointStatePublisherWidget;
  using super = qt::Widget;

public:
  explicit JointStatePublisherWidget(SetupAssistant* main);

  void updateInternalDataStructures();

private Q_SLOTS:
  void onValueChanged(double value, const std::string& jnt_name);
  void onCenterButtonClicked();
  void onRandomButtonClicked();

private:
  SetupAssistant* main_;
  QVBoxLayout* rows_;
  QVBoxLayout* slider_rows_;
  QVBoxLayout* button_rows_;

  sensor_msgs::msg::JointState js_;
  std::vector<qt::DoubleSliderDisplay*> sliders_;

  ros2::PublisherPtr<sensor_msgs::msg::JointState> js_pub_;
  ros2::PublisherPtr<moveit_msgs::msg::DisplayRobotState> drs_pub_;

  ros2::TimerPtr publish_timer_;

  void publish();
};
}  // namespace setup_assistant
}  // namespace gui
