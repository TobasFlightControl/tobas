#pragma once

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "tobas_setup_assistant/param_getters/double_range.hpp"
#include "tobas_setup_assistant/param_getters/vector3d.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace fixed_wing
{
class VehicleParametersWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = VehicleParametersWidget;
  using super = BaseSelectedLinkSettingWidget;

public:
  explicit VehicleParametersWidget();

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double wingSurface() const;
  double wingSpan() const;
  double mac() const;
  Eigen::Vector3d aerodynamicCenter() const;
  tobas_std::Range<double> alphaLimit() const;

private:
  ParamGetterWidget_DoubleSpinBox* wing_surface_;
  ParamGetterWidget_DoubleSpinBox* wing_span_;
  ParamGetterWidget_DoubleSpinBox* mac_;
  ParamGetterWidget_Vector3d* aerodynamic_center_;
  ParamGetterWidget_DoubleRange* alpha_limit_;
};
}  // namespace fixed_wing
}  // namespace sa
}  // namespace gui
