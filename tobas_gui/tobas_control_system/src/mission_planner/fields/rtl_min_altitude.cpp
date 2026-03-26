#include "tobas_control_system/mission_planner/fields/rtl_min_altitude.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
RtlMinAltitudeWidget::RtlMinAltitudeWidget()
{
  spin_box_ = new tobas::qt::DoubleSpinBox();
  spin_box_->setDecimals(2);
  spin_box_->setMinimum(0.);
  spin_box_->setMaximum(150.);  // 日本の飛行禁止空域
  spin_box_->setValue(15.);     // https://ardupilot.org/copter/docs/rtl-mode.html
  spin_box_->setSuffix(" m");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* RtlMinAltitudeWidget::label() const
{
  return "Minimum Altitude (wrt. Home)";
}

double RtlMinAltitudeWidget::getValue() const
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
}  // namespace tobas
