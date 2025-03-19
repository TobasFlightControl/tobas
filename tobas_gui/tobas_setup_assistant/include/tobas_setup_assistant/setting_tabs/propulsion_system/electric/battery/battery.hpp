#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class BatteryWidget : public QWidget
{
  Q_OBJECT

  using self = BatteryWidget;
  using super = QWidget;

  static constexpr char kTypeKey[] = "battery_type";

public:
  explicit BatteryWidget();

  bool isValid();

  YAML::Node dump();
  void load(const YAML::Node& node);

  double nominalVoltage();
  double maxVoltage();
  double sagVoltage();
  double maxCurrent();
  double capacity();
  double internalRegistance();

private:
  qt::ComboBox* type_;
  qt::StackedWidget* batteries_;

  BatteryWidget_Base* widget(int index);

  BatteryWidget_Base* selected();
  const BatteryWidget_Base* selected() const;
};
};  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
