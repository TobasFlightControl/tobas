#pragma once

#include "../param_getters/double_spin_box.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/vector3d.hpp"
#include "./base_setting.hpp"

namespace gui
{
namespace sa
{
class BarometerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = BarometerWidget;
  using super = BaseSettingWidget;

public:
  explicit BarometerWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  Eigen::Vector3d offset() const;
  int updateRate() const;
  double pressureVariance() const;

private:
  ParamGetterWidget_Vector3d* offset_;
  ParamGetterWidget_SpinBox* update_rate_;
  ParamGetterWidget_DoubleSpinBox* pressure_var_;
};
};  // namespace sa
}  // namespace gui
