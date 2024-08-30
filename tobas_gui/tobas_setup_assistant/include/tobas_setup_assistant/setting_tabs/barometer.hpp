#pragma once

#include "./base_setting.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/double_spin_box.hpp"
#include "../param_getters/vector3d.hpp"

namespace gui
{
namespace setup_assistant
{
class BarometerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = BarometerWidget;
  using super = BaseSettingWidget;

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

private:
  ParamGetterWidget_Vector3d* offset_;
  ParamGetterWidget_SpinBox* update_rate_;
  ParamGetterWidget_DoubleSpinBox* pressure_var_;
};
};  // namespace setup_assistant
}  // namespace gui
