#pragma once

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class TransmissionWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = TransmissionWidget;
  using super = BaseSelectedLinkSettingWidget;

public:
  explicit TransmissionWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double gearRatio() const;

private:
  ParamGetterWidget_DoubleSpinBox* gear_ratio_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
