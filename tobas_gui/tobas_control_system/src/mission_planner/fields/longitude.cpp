#include "tobas_control_system/mission_planner/fields/longitude.hpp"

#include <QHBoxLayout>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
LongitudeWidget::LongitudeWidget()
{
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(9);
  spin_box_->setMinimum(-180.);
  spin_box_->setMaximum(180.);
  spin_box_->setValue(0.);
  spin_box_->setSuffix(" deg");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* LongitudeWidget::label() const
{
  return "Longitude";
}

double LongitudeWidget::getValue() const
{
  return spin_box_->value();
}

void LongitudeWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
