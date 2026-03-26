#include "tobas_control_system/mission_planner/fields/max_heading_rate.hpp"

#include <QHBoxLayout>

#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
MaxHeadingRateWidget::MaxHeadingRateWidget()
{
  // https://docs.px4.io/main/en/advanced_config/parameter_reference#MPC_YAWRAUTO_MAX
  spin_box_ = new tobas::qt::SpinBox();
  spin_box_->setMinimum(5);
  spin_box_->setMaximum(360);
  spin_box_->setValue(60);
  spin_box_->setSuffix(" deg/s");

  const auto cols = new QHBoxLayout();
  setLayout(cols);
  cols->addWidget(spin_box_);

  connect(spin_box_, QOverload<int>::of(&QSpinBox::valueChanged), this, &BaseFieldWidget::updated);
}

const char* MaxHeadingRateWidget::label() const
{
  return "Maximum Heading Rate";
}

double MaxHeadingRateWidget::getValue() const
{
  return tbs::deg2rad(spin_box_->value());
}

void MaxHeadingRateWidget::setValue(double value)
{
  spin_box_->setValue(tbs::rad2deg(value));
}
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
