#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_setup_assistant/param_getters/double_range.hpp"
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
class VehicleParametersWidget : public QWidget
{
  Q_OBJECT

  using self = VehicleParametersWidget;
  using super = QWidget;

public:
  explicit VehicleParametersWidget();

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  double wingSurface() const;
  double wingSpan() const;
  double mac() const;
  Eigen::Vector3d aerodynamicCenter() const;
  tbs::Range<double> alphaLimit() const;

private:
  ParamGetterWidget_DoubleSpinBox* wing_surface_;
  ParamGetterWidget_DoubleSpinBox* wing_span_;
  ParamGetterWidget_DoubleSpinBox* mac_;
  ParamGetterWidget_Vector3d* aerodynamic_center_;
  ParamGetterWidget_DoubleRange* alpha_limit_;
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
