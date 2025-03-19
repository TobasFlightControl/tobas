#pragma once

#include "./base_setting.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/double_spin_box.hpp"
#include "../param_getters/vector3d.hpp"

namespace gui
{
namespace sa
{
class IMUWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = IMUWidget;
  using super = BaseSettingWidget;

public:
  explicit IMUWidget();

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
  double gyroNoiseDensity() const;
  double gyroOffsetNorm() const;
  double gyroRandomWalk() const;
  int gyroBiasCorrTime() const;
  double accNoiseDensity() const;
  double accOffsetNorm() const;
  double accRandomWalk() const;
  int accBiasCorrTime() const;

private:
  ParamGetterWidget_Vector3d* offset_;
  ParamGetterWidget_SpinBox* update_rate_;
  ParamGetterWidget_SpinBox* gyro_noise_density_;
  ParamGetterWidget_SpinBox* gyro_offset_norm_;
  ParamGetterWidget_DoubleSpinBox* gyro_random_walk_;
  ParamGetterWidget_SpinBox* gyro_bias_corr_time_;
  ParamGetterWidget_SpinBox* acc_noise_density_;
  ParamGetterWidget_SpinBox* acc_offset_norm_;
  ParamGetterWidget_DoubleSpinBox* acc_random_walk_;
  ParamGetterWidget_SpinBox* acc_bias_corr_time_;
};
};  // namespace sa
}  // namespace gui
