#pragma once

#include <tobas_qt_tools/widgets/circle_widget.hpp>
#include <tobas_qt_tools/widgets/toggle_switch.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

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

public:
  explicit TogglesViewer(const RosQtBridge& bridge);

  void reset();

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  qt::ToggleSwitch* kill_;
  qt::ToggleSwitch* sub_mode_;

  qt::CircleWidget* acrobat_mode_;
  qt::CircleWidget* stabilize_mode_;
  qt::CircleWidget* loiter_mode_;

  void setToggleSwitchPointSizes();
  void setFlightModePointSizes();

private Q_SLOTS:
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
