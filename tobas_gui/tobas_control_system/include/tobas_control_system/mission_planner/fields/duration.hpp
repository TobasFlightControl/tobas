#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace gcs
{
namespace field
{
class DurationWidget : public BaseField
{
public:
  explicit DurationWidget();

  const char* label() const override;

  int value() const;
  void setValue(int value);

private:
  qt::SpinBox* spinbox_;
};
}  // namespace field
}  // namespace gcs
}  // namespace gui
