#include <tobas_string_tools/core.hpp>

#include "tobas_setup_assistant/param_getters/linear_equation.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_LinearEquation::ParamGetterWidget_LinearEquation(
  const QString& param_name,
  const QString& description_text,
  const QString& left,
  const QString& value)
  : super(param_name, description_text)
{
  c0_ = new qt::DoubleSpinBox();
  c1_ = new qt::DoubleSpinBox();
  suffix_ = new QLabel();

  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  cols->addWidget(new QLabel(left + " = "));
  cols->addWidget(c1_);
  cols->addWidget(new QLabel(value + " + "));
  cols->addWidget(c0_);
  cols->addWidget(suffix_);
  cols->addStretch();

  connect(c0_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
  connect(c1_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

ParamGetterWidget_LinearEquation::ValueType ParamGetterWidget_LinearEquation::getValue() const
{
  return { c0_->value(), c1_->value() };
}

bool ParamGetterWidget_LinearEquation::setValue(const ValueType& src)
{
  c0_->setValue(src.first);
  c1_->setValue(src.second);

  return true;
}

void ParamGetterWidget_LinearEquation::setDecimals(int decimals)
{
  c0_->setDecimals(decimals);
  c1_->setDecimals(decimals);
}

void ParamGetterWidget_LinearEquation::setSuffix(const QString& suffix)
{
  suffix_->setText(QString::fromStdString(str::convertToSuperscript(suffix.toStdString())));
}

void ParamGetterWidget_LinearEquation::onValueChanged()
{
  Q_EMIT valueChanged(getValue());
}
}  // namespace sa
}  // namespace gui
