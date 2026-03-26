#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class BatteryWidget : public tobas::qt::ScrollArea
{
  Q_OBJECT

  using self = BatteryWidget;
  using super = tobas::qt::ScrollArea;

  static constexpr char kTypeKey[] = "battery_type";

public:
  explicit BatteryWidget();

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  double nominalVoltage();
  double maxVoltage();
  double sagVoltage();
  double maxCurrent();
  double capacity();
  double internalRegistance();

private:
  tobas::qt::ComboBox* type_;
  tobas::qt::StackedWidget* batteries_;

  BatteryWidget_Base* widget(int index);
  const BatteryWidget_Base* widget(int index) const;

  BatteryWidget_Base* selected();
  const BatteryWidget_Base* selected() const;
};
};  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
