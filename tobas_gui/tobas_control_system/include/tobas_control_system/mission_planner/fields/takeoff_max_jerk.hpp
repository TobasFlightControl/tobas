#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
class TakeoffMaxJerkWidget : public FieldWidget<double>
{
public:
  explicit TakeoffMaxJerkWidget();

  const char* label() const override;

  double getValue() const override;
  void setValue(double value) override;

private:
  tobas::qt::DoubleSpinBox* spin_box_;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
