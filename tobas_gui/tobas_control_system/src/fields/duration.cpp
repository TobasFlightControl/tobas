#include <QHBoxLayout>

#include "tobas_control_system/fields/duration.hpp"

namespace gui
{
namespace control_system
{
namespace field
{
DurationWidget::DurationWidget()
{
  spinbox_ = new qt::SpinBox();
  spinbox_->setMinimum(1);
  spinbox_->setValue(10);
  spinbox_->setSuffix(" s");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spinbox_);

  connect(spinbox_, QOverload<int>::of(&QSpinBox::valueChanged), this, &BaseField::updated);
}

const char* DurationWidget::label() const
{
  return "Duration";
}

int DurationWidget::value() const
{
  return spinbox_->value();
}

void DurationWidget::setValue(int value)
{
  spinbox_->setValue(value);
}
}  // namespace field
}  // namespace control_system
}  // namespace gui
