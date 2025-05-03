#include <tobas_string_tools/core.hpp>

#include "tobas_setup_assistant/param_getters/spin_box.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_SpinBox::ParamGetterWidget_SpinBox(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  spin_box_ = new qt::SpinBox();
  rows_->addWidget(spin_box_);
  connect(spin_box_, QOverload<int>::of(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
}

int ParamGetterWidget_SpinBox::getValue() const
{
  return spin_box_->value();
}

bool ParamGetterWidget_SpinBox::setValue(const int& src)
{
  if (src < spin_box_->minimum() || spin_box_->maximum() < src) {
    return false;
  }

  spin_box_->setValue(src);
  return true;
}

void ParamGetterWidget_SpinBox::setMinimum(int minimum)
{
  spin_box_->setMinimum(minimum);
}

void ParamGetterWidget_SpinBox::setMaximum(int maximum)
{
  spin_box_->setMaximum(maximum);
}

void ParamGetterWidget_SpinBox::setSingleStep(int single_step)
{
  spin_box_->setSingleStep(single_step);
}

void ParamGetterWidget_SpinBox::setSuffix(const QString& suffix)
{
  spin_box_->setSuffix(QString::fromStdString(str::convertToSuperscript(suffix.toStdString())));
}

void ParamGetterWidget_SpinBox::onValueChanged(int value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace sa
}  // namespace gui
