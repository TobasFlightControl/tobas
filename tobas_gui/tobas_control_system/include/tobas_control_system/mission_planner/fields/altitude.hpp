#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace gui
{
namespace gcs
{
namespace field
{
class AltitudeWidget : public BaseField
{
public:
  explicit AltitudeWidget();

  const char* label() const override;

  double value() const;
  void setValue(double value);

private:
  qt::DoubleSpinBox* spinbox_;
};
}  // namespace field
}  // namespace gcs
}  // namespace gui
