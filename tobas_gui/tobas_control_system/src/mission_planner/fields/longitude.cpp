#include "tobas_control_system/mission_planner/fields/longitude.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace gcs
{
namespace field
{
LongitudeWidget::LongitudeWidget()
{
  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setDecimals(9);
  spinbox_->setMinimum(-180.);
  spinbox_->setMaximum(180.);
  spinbox_->setValue(0.);
  spinbox_->setSuffix(" deg");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spinbox_);

  connect(spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseField::updated);
}

const char* LongitudeWidget::label() const
{
  return "Longitude";
}

double LongitudeWidget::value() const
{
  return spinbox_->value();
}

void LongitudeWidget::setValue(double value)
{
  spinbox_->setValue(value);
}
}  // namespace field
}  // namespace gcs
}  // namespace gui
