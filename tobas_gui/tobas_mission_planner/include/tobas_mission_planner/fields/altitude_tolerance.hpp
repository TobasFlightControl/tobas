#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
class AltitudeToleranceWidget : public BaseField
{
public:
  explicit AltitudeToleranceWidget();

  const char* label() const override;

  double value() const;
  void setValue(double value);

private:
  qt::DoubleSpinBox* spinbox_;
};
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
