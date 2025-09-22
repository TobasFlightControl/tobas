#pragma once

#include <QButtonGroup>
#include <rclcpp/rclcpp.hpp>

#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./base.hpp"

namespace gui
{
namespace sc
{
class MagCalibrationWidget : public BaseWidget
{
  Q_OBJECT

  using self = MagCalibrationWidget;
  using super = BaseWidget;

  static constexpr int kDefaultIndex = 0;

public:
  explicit MagCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  const char* title() const override;

  void reset() override;

  void setNamespace(const std::string& ns);

private:
  QButtonGroup* btn_group_;
  qt::StackedWidget* stack_;

  void addMagCalibWidget(BaseMagCalibWidget* widget, const QString& label, int id);

  BaseMagCalibWidget* getWidget(int index);

  void setCurrentIndex(int index);
};
}  // namespace sc
}  // namespace gui
