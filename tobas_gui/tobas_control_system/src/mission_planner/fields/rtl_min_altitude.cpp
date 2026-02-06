#include "tobas_control_system/mission_planner/fields/rtl_min_altitude.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace ctrl
{
namespace field
{
RtlMinAltitudeWidget::RtlMinAltitudeWidget()
{
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(2);
  spin_box_->setValue(15.);  // https://ardupilot.org/copter/docs/rtl-mode.html
  spin_box_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* RtlMinAltitudeWidget::label() const
{
  return "Minimum Altitude (wrt. Home)";
}

double RtlMinAltitudeWidget::value() const
{
  return spin_box_->value();
}

void RtlMinAltitudeWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
