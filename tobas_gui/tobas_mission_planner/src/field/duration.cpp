#include <QHBoxLayout>

#include "tobas_mission_planner/fields/duration.hpp"

namespace gui
{
namespace mission_planner
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
}  // namespace field
}  // namespace mission_planner
}  // namespace gui
