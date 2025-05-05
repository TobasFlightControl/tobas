#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>

#include <tobas_msgs/msg/cpu.hpp>

namespace gui
{
namespace gcs
{
class CPUViewerWidget : public QWidget
{
  Q_OBJECT

  using self = CPUViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 40;

  static constexpr double kMinTemp = 0.;    // [degC]
  static constexpr double kMaxTemp = 100.;  // [degC]
  static constexpr double kMinLoad = 0.;    // [%]
  static constexpr double kMaxLoad = 100.;  // [%]

public:
  explicit CPUViewerWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  qt::HPositionBarWidget* temp_;
  qt::HPositionBarWidget* load_;

  ros2::SubscriberPtr<tobas_msgs::msg::Cpu> cpu_sub_;

  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
};
}  // namespace gcs
}  // namespace gui
