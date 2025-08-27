#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace ctrl
{
namespace field
{
class LatitudeWidget : public BaseField
{
public:
  explicit LatitudeWidget();

  const char* label() const override;

  double value() const;
  void setValue(double value);

private:
  qt::DoubleSpinBox* spin_box_;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
