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
  bool isValid() const;
  // getters
  bool symmetric() const;
  double c_root() const;
  double c_tip() const;
  double span() const;
  double oswaldEfficiency() const;
  double c_lift_0() const;
  double c_drag_0() const;
  double c_pitch_0() const;
  double sweepBack() const;
  Eigen::Vector3d position() const;
  Eigen::Vector3d rotation() const;
  // properties
  double surfaceArea() const;
  double aspectRatio() const;
  double c_mac() const;
  double y_mac() const;
  Eigen::Vector3d mac_position() const; // wing座標系(flu)でみたときのmacの位置
  double c_lift_alpha() const;
  double c_lift(const double& alpha) const;
  double c_drag(const double& alpha) const;
  double c_roll_beta(const double& alpha) const;
  double c_roll_p() const;
  double c_roll_r(const double& alpha) const;
  double c_roll(const double& V, const double& alpha, const double& beta, const double& p, const double& r) const;
  double c_yaw_p(const double& alpha) const;
  double c_yaw_r(const double& alpha) const;
  double c_yaw(const double& V, const double& alpha, const double& p, const double& r) const;

private:
  QCheckBox* symmetric_; // 対称でないときは翼の右側が残ることにする
  ParamGetterWidget_DoubleSpinBox* c_root_;
  ParamGetterWidget_DoubleSpinBox* c_tip_;
  ParamGetterWidget_DoubleSpinBox* span_;
  ParamGetterWidget_DoubleSpinBox* oswald_efficiency_;
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
