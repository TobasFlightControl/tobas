#pragma once

#include <QWidget>
#include <QCheckBox>

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "tobas_setup_assistant/param_getters/vector3d.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace hp
{
class WingWidget : public QWidget
{
  Q_OBJECT

  using self = WingWidget;
  using super = QWidget;

public:
  explicit WingWidget();
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();
  bool symmetric();
  double c_root();
  double c_tip();
  double span();
  double c_lift_0();
  double c_drag_0();
  double c_pitch_0();
  double sweep_back();
  Eigen::Vector3d position();
  Eigen::Vector3d rotation();

private:
  QCheckBox* symmetric_;
  ParamGetterWidget_DoubleSpinBox* c_root_;
  ParamGetterWidget_DoubleSpinBox* c_tip_;
  ParamGetterWidget_DoubleSpinBox* span_;
  ParamGetterWidget_DoubleSpinBox* c_lift_0_;
  ParamGetterWidget_DoubleSpinBox* c_drag_0_;
  ParamGetterWidget_DoubleSpinBox* c_pitch_0_;
  ParamGetterWidget_DoubleSpinBox* sweep_back_;
  ParamGetterWidget_Vector3d* position_;
  ParamGetterWidget_Vector3d* rotation_;
};
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
