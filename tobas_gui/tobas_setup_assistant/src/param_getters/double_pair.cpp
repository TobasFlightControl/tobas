#include "tobas_setup_assistant/param_getters/double_pair.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_DoublePair::ParamGetterWidget_DoublePair(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  first_ = new qt::DoubleSpinBox();
  cols->addWidget(first_);

  second_ = new qt::DoubleSpinBox();
  cols->addWidget(second_);

  connect(first_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(second_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

std::pair<double, double> ParamGetterWidget_DoublePair::getValue() const
{
  return { first(), second() };
}

bool ParamGetterWidget_DoublePair::setValue(const std::pair<double, double>& src)
{
  first_->setValue(src.first);
  second_->setValue(src.second);
  return true;
}

void ParamGetterWidget_DoublePair::setDecimals(int decimals)
{
  first_->setDecimals(decimals);
  second_->setDecimals(decimals);
}

void ParamGetterWidget_DoublePair::setMinimum(double minimum)
{
  first_->setMinimum(minimum);
  second_->setMinimum(minimum);
}

void ParamGetterWidget_DoublePair::setMaximum(double maximum)
{
  first_->setMaximum(maximum);
  second_->setMaximum(maximum);
}

void ParamGetterWidget_DoublePair::setSingleStep(double single_step)
{
  first_->setSingleStep(single_step);
  second_->setSingleStep(single_step);
}

void ParamGetterWidget_DoublePair::setSuffix(const QString& suffix)
{
  first_->setSuffix(suffix);
  second_->setSuffix(suffix);
}

double ParamGetterWidget_DoublePair::first() const
{
  return first_->value();
}

double ParamGetterWidget_DoublePair::second() const
{
  return second_->value();
}

void ParamGetterWidget_DoublePair::onValueChanged(double)
{
  Q_EMIT valueChanged({ first(), second() });
}
}  // namespace sa
}  // namespace gui
