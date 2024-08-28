#pragma once

#include "./optional_device.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/double_spin_box.hpp"
#include "../param_getters/vector3d.hpp"

namespace gui
{
namespace setup_assistant
{
class SetupAssistant;

class GpsWidget : public OptionalDeviceWidget
{
  Q_OBJECT

  using self = GpsWidget;
  using super = OptionalDeviceWidget;

public:
  using super::OptionalDeviceWidget;

  const char* name() override;
  const char* title() override;
  const char* description() override;

  void onInit() override;
  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

protected:
  bool defaultEquipped() const override;

private:
  ParamGetterWidget_Vector3d* offset_;
  ParamGetterWidget_SpinBox* update_rate_;
  ParamGetterWidget_DoubleSpinBox* delay_;
  ParamGetterWidget_SpinBox* pos_corr_time_;
  ParamGetterWidget_DoubleSpinBox* horizontal_pos_accuracy_;
  ParamGetterWidget_DoubleSpinBox* vertical_pos_accuracy_;
  ParamGetterWidget_DoubleSpinBox* horizontal_vel_stddev_;
  ParamGetterWidget_DoubleSpinBox* vertical_vel_stddev_;
};
};  // namespace setup_assistant
}  // namespace gui
