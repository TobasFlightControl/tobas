#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
class MaxHeadingAccelWidget : public FieldWidget<double>
{
public:
  explicit MaxHeadingAccelWidget();

  const char* label() const override;

  double getValue() const override;
  void setValue(double value) override;

private:
  qt::SpinBox* spin_box_;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
