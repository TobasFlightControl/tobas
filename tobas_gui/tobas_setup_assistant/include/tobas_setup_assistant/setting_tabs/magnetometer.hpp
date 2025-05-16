#pragma once

#include "../param_getters/spin_box.hpp"
#include "../param_getters/vector3d.hpp"
#include "./base_setting.hpp"

namespace gui
{
namespace sa
{
class MagnetometerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = MagnetometerWidget;
  using super = BaseSettingWidget;

public:
  explicit MagnetometerWidget();

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
  int noiseStddev() const;
  int hardBiasNorm() const;

private:
  ParamGetterWidget_Vector3d* offset_;
  ParamGetterWidget_SpinBox* update_rate_;
  ParamGetterWidget_SpinBox* noise_stddev_;
  ParamGetterWidget_SpinBox* hard_bias_norm_;
};
};  // namespace sa
}  // namespace gui
