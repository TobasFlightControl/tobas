#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace ctrl
{
namespace field
{
class RtlMinAltitudeWidget : public FieldWidget<double>
{
public:
  explicit RtlMinAltitudeWidget();

  const char* label() const override;

  double getValue() const override;
  void setValue(double value) override;

private:
  qt::DoubleSpinBox* spin_box_;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
