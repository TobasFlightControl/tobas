#pragma once

#include <tobas_qt_tools/widgets/circle_widget.hpp>
#include <tobas_qt_tools/widgets/toggle_switch.hpp>
#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs_adapter/rc_input.hpp>

namespace gui
{
namespace gcs
{
namespace rcin
{
class TogglesViewer : public QWidget
{
  Q_OBJECT

  using self = TogglesViewer;
  using super = QWidget;

  static constexpr auto kOffColor = Qt::gray;
  static constexpr auto kOnColorEnable = Qt::green;
  static constexpr auto kOnColorDisable = Qt::darkGray;

Q_SIGNALS:
  void rcInputReceived(bool enable, bool kill, uint8_t mode, bool sub_mode);

public:
  explicit TogglesViewer(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  const rclcpp::Node::SharedPtr node_;

  qt::ToggleSwitch* kill_;
  qt::ToggleSwitch* sub_mode_;

  qt::CircleWidget* acrobat_mode_;
  qt::CircleWidget* stabilize_mode_;
  qt::CircleWidget* loiter_mode_;

  ros2::SubscriberPtr<tobas_msgs::msg::RCInput> rcin_sub_;

  void setToggleSwitchPointSizes();
  void setFlightModePointSizes();

  void rcInputCbRos(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin);

private Q_SLOTS:
  void rcInputCbQt(bool enable, bool kill, uint8_t _mode, bool sub_mode);
};
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
