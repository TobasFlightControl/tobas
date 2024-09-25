#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
class AcceptanceRadiusWidget : public BaseField
{
public:
  explicit AcceptanceRadiusWidget();

  const char* label() const override;

  double value() const;

private:
  qt::DoubleSpinBox* spinbox_;
};
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
