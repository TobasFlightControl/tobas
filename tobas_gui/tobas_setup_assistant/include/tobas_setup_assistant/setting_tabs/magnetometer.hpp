#pragma once

#include "./base_setting.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/vector3d.hpp"

namespace gui
{
namespace setup_assistant
{
class MagnetometerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = MagnetometerWidget;
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

  Eigen::Vector3d offset() const;
  int updateRate() const;
  int gaussNoise() const;
  int uniformNoise() const;

private:
  ParamGetterWidget_Vector3d* offset_;
  ParamGetterWidget_SpinBox* update_rate_;
  ParamGetterWidget_SpinBox* gauss_noise_;
  ParamGetterWidget_SpinBox* uniform_noise_;
};
};  // namespace setup_assistant
}  // namespace gui
