#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/string.hpp>

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_DoubleSpinBox::ParamGetterWidget_DoubleSpinBox(
  const QString& param_name,
  const QString& description_text)
  : super(param_name, description_text)
{
  spin_box_ = new qt::DoubleSpinBox();
  rows_->addWidget(spin_box_);

  spin_box_->setFocusPolicy(Qt::StrongFocus);
  connect(spin_box_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

double ParamGetterWidget_DoubleSpinBox::getValue() const
{
  return spin_box_->value();
}

bool ParamGetterWidget_DoubleSpinBox::setValue(const double& src)
{
  if (src < spin_box_->minimum() || spin_box_->maximum() < src)
    return false;

  spin_box_->setValue(src);
  return true;
}

void ParamGetterWidget_DoubleSpinBox::setDecimals(int decimals)
{
  spin_box_->setDecimals(decimals);
}

void ParamGetterWidget_DoubleSpinBox::setMinimum(double minimum)
{
  spin_box_->setMinimum(minimum);
}

void ParamGetterWidget_DoubleSpinBox::setMaximum(double maximum)
{
  spin_box_->setMaximum(maximum);
}

void ParamGetterWidget_DoubleSpinBox::setSingleStep(double single_step)
{
  spin_box_->setSingleStep(single_step);
}

void ParamGetterWidget_DoubleSpinBox::setSuffix(const QString& suffix)
{
  spin_box_->setSuffix(QString::fromStdString(tobas_std::convertToSuperscript(suffix.toStdString())));
}

void ParamGetterWidget_DoubleSpinBox::onValueChanged(double value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace setup_assistant
}  // namespace gui
