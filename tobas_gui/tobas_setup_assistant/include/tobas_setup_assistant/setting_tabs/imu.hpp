#pragma once

#include "./base_setting.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/double_spin_box.hpp"
#include "../param_getters/vector3d.hpp"

namespace gui
{
namespace setup_assistant
{
class IMUWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = IMUWidget;
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
  ParamGetterWidget_DoubleSpinBox* gyro_noise_density_;
  ParamGetterWidget_DoubleSpinBox* gyro_random_walk_;
  ParamGetterWidget_SpinBox* gyro_bias_corr_time_;
  ParamGetterWidget_DoubleSpinBox* gyro_turn_on_bias_sigma_;
  ParamGetterWidget_SpinBox* gyro_lpf_cutoff_freq_;
  ParamGetterWidget_DoubleSpinBox* acc_noise_density_;
  ParamGetterWidget_DoubleSpinBox* acc_random_walk_;
  ParamGetterWidget_SpinBox* acc_bias_corr_time_;
  ParamGetterWidget_DoubleSpinBox* acc_turn_on_bias_sigma_;
  ParamGetterWidget_SpinBox* acc_lpf_cutoff_freq_;
};
};  // namespace setup_assistant
}  // namespace gui
