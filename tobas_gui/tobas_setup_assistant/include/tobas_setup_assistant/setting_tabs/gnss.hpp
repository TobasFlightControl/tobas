#pragma once

#include "./optional_device.hpp"
#include "../param_getters/spin_box.hpp"
#include "../param_getters/double_spin_box.hpp"
#include "../param_getters/vector3d.hpp"

namespace gui
{
namespace sa
{
class GNSSWidget : public OptionalDeviceWidget
{
  Q_OBJECT

  using self = GNSSWidget;
  using super = OptionalDeviceWidget;

public:
  explicit GNSSWidget();

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
  double delay() const;
  int positionCorrectionTime() const;
  double horizontalPositionAccuracy() const;
  double verticalPositionAccuracy() const;
  double horizontalVelocityStddev() const;
  double verticalVelocityStddev() const;

protected:
  bool defaultEquipped() const override;

private:
  ParamGetterWidget_Vector3d* offset_;
  ParamGetterWidget_SpinBox* update_rate_;
  ParamGetterWidget_SpinBox* delay_;
  ParamGetterWidget_SpinBox* pos_corr_time_;
  ParamGetterWidget_DoubleSpinBox* horizontal_pos_accuracy_;
  ParamGetterWidget_DoubleSpinBox* vertical_pos_accuracy_;
  ParamGetterWidget_DoubleSpinBox* horizontal_vel_stddev_;
  ParamGetterWidget_DoubleSpinBox* vertical_vel_stddev_;
};
};  // namespace sa
}  // namespace gui
