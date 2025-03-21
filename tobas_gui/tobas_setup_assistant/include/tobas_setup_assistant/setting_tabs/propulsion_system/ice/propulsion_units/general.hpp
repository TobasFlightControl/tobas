#pragma once

#include <tobas_drone_core/propulsion_system/turning_direction.hpp>

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "tobas_setup_assistant/param_getters/combo_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class GeneralWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = GeneralWidget;
  using super = BaseSelectedLinkSettingWidget;

  static constexpr char kCWName[] = "CW";
  static constexpr char kCCWName[] = "CCW";

public:
  explicit GeneralWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  int channel() const;
  tobas::turning_direction_t direction() const;

private:
  ParamGetterWidget_SpinBox* channel_;
  ParamGetterWidget_ComboBox* direction_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
