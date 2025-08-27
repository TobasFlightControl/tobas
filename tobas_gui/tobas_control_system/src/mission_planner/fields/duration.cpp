#include "tobas_control_system/mission_planner/fields/duration.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace ctrl
{
namespace field
{
DurationWidget::DurationWidget()
{
  spin_box_ = new qt::SpinBox();
  spin_box_->setMinimum(1);
  spin_box_->setValue(10);
  spin_box_->setSuffix(" s");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<int>::of(&QSpinBox::valueChanged), this, &BaseField::updated);
}

const char* DurationWidget::label() const
{
  return "Duration";
}

int DurationWidget::value() const
{
  return spin_box_->value();
}

void DurationWidget::setValue(int value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
