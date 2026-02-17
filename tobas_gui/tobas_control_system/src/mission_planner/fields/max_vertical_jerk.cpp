#include "tobas_control_system/mission_planner/fields/max_vertical_jerk.hpp"

#include <QHBoxLayout>

namespace gui
{
namespace ctrl
{
namespace field
{
MaxVerticalJerkWidget::MaxVerticalJerkWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_JERK_AUTO
  spin_box_ = new qt::DoubleSpinBox();
  spin_box_->setDecimals(1);
  spin_box_->setMinimum(1.);
  spin_box_->setMaximum(80.);
  spin_box_->setValue(4.);
  spin_box_->setSuffix(" m/s³");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* MaxVerticalJerkWidget::label() const
{
  return "Maximum Vertical Jerk";
}

double MaxVerticalJerkWidget::getValue() const
{
  return spin_box_->value();
}

void MaxVerticalJerkWidget::setValue(double value)
{
  spin_box_->setValue(value);
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
