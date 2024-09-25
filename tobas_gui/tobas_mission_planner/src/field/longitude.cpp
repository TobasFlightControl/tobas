#include <QHBoxLayout>

#include "tobas_mission_planner/fields/longitude.hpp"

namespace gui
{
namespace mission_planner
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
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
