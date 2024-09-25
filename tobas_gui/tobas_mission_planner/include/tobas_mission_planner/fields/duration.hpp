#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace mission_planner
{
namespace field
{
class DurationWidget : public BaseField
{
public:
  explicit DurationWidget();

  const char* label() const override;

  int value() const;

private:
  qt::SpinBox* spinbox_;
};
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
