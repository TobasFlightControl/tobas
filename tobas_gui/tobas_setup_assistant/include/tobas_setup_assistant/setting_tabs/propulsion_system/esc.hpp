#pragma once

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class ESCWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit ESCWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double maxCurrent() const;

private:
  ParamGetterWidget_SpinBox* max_current_;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
