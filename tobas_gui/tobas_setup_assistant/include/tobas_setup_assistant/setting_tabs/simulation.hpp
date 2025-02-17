#pragma once

#include "./base_setting.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/double_spin_box.hpp"

namespace gui
{
namespace sa
{
class SimulationWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = SimulationWidget;
  using super = BaseSettingWidget;

public:
  explicit SimulationWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  double latitudeZero() const;
  double longitudeZero() const;
  double altitudeZero() const;
  double maxModelErrorRate() const;

private:
  ParamGetterWidget_DoubleSpinBox* latitude_zero_;
  ParamGetterWidget_DoubleSpinBox* longitude_zero_;
  ParamGetterWidget_DoubleSpinBox* altitude_zero_;
  ParamGetterWidget_SpinBox* max_model_error_rate_;
};
};  // namespace sa
}  // namespace gui
