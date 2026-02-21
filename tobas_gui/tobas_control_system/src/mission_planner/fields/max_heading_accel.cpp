#include "tobas_control_system/mission_planner/fields/max_heading_accel.hpp"

#include <QHBoxLayout>

#include <tobas_std_tools/unit_conversions.hpp>

namespace gui
{
namespace ctrl
{
namespace field
{
MaxHeadingAccelWidget::MaxHeadingAccelWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_YAWRAUTO_ACC
  spin_box_ = new qt::SpinBox();
  spin_box_->setMinimum(5);
  spin_box_->setMaximum(360);
  spin_box_->setValue(120);  // 20 deg/s^2 -> 120 deg/s^2
  spin_box_->setSuffix(" deg/s²");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<int>::of(&QSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* MaxHeadingAccelWidget::label() const
{
  return "Maximum Heading Acceleration";
}

double MaxHeadingAccelWidget::getValue() const
{
  return tbs::deg2rad(spin_box_->value());
}

void MaxHeadingAccelWidget::setValue(double value)
{
  spin_box_->setValue(tbs::rad2deg(value));
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
