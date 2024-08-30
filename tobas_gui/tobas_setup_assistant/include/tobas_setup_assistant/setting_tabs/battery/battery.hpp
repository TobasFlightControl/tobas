#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class BatteryWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = BatteryWidget;
  using super = BaseSettingWidget;

  static constexpr char kTypeKey[] = "battery_type";

public:
  using super::BaseSettingWidget;

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onInit() override;
  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  double nominalVoltage();
  double maxVoltage();
  double sagVoltage();
  double maxCurrent();
  double capacity();
  double internalRegistance();

private:
  qt::ComboBox* type_;
  qt::StackedWidget* batteries_;

  BatteryWidget_Base* selected();
};
};  // namespace setup_assistant
}  // namespace gui
