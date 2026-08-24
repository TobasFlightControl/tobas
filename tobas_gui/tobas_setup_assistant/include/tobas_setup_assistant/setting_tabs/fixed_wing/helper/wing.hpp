#pragma once

#include <yaml-cpp/yaml.h>
#include <QCheckBox>
#include <QWidget>

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

  static constexpr char kSymmetricLabel[] = "symmetric";
  static constexpr char kCRootLabel[] = "c_root";
  static constexpr char kCTipLabel[] = "c_tip";
  static constexpr char kSpanLabel[] = "span";
  static constexpr char kOswaldLabel[] = "oswald efficiency";
  static constexpr char kCLift0Label[] = "c_lift_0";
  static constexpr char kCDrag0Label[] = "c_drag_0";
  static constexpr char kCPitch0Label[] = "c_pitch_0";
  static constexpr char kSweepBackLabel[] = "sweep_back";
  static constexpr char kPositionLabel[] = "position";
  static constexpr char kRotationLabel[] = "rotation";

public:
  explicit WingWidget();
  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid() const;

  YAML::Node dump() const;
  void load(const YAML::Node& node);

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
  // setters
  void symmetric(const bool& value);
  void c_root(const double& value);
  void c_tip(const double& value);
  void span(const double& value);
  void oswaldEfficiency(const double& value);
  void c_lift_0(const double& value);
  void c_drag_0(const double& value);
  void c_pitch_0(const double& value);
  void sweepBack(const double& value);
  void position(const Eigen::Vector3d& value);
  void rotation(const Eigen::Vector3d& value);
  // properties
  double surfaceArea() const;
  double aspectRatio() const;
  double c_mac() const;
  double y_mac() const;
  Eigen::Vector3d mac_position() const;  // wing座標系(flu)でみたときのmacの位置
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
  QCheckBox* symmetric_;  // 対称でないときは翼の右側が残ることにする
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
