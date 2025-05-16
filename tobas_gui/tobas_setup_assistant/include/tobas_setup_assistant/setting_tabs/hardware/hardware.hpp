#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
class HardwareWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = HardwareWidget;
  using super = BaseSettingWidget;

  static constexpr char kTypeKey[] = "hardware_type";

public:
  explicit HardwareWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  const char* hardwarePackage() const;

private Q_SLOTS:
  void setCurrentHardware(int index);

private:
  qt::ComboBox* type_;
  qt::StackedWidget* hardwares_;
  qt::DescriptionWidget* description_;

  BaseHardwareWidget* selected();
  const BaseHardwareWidget* selected() const;
};
};  // namespace sa
}  // namespace gui
