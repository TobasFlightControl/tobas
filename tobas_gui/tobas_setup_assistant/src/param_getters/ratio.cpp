#include "tobas_setup_assistant/param_getters/ratio.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_Ratio::ParamGetterWidget_Ratio(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  left_value_ = new qt::SpinBox();
  right_value_ = new qt::SpinBox();
  left_text_ = new QLabel();
  right_text_ = new QLabel();

  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  cols->addWidget(left_text_);
  cols->addWidget(left_value_);
  cols->addWidget(new QLabel(" : "));
  cols->addWidget(right_value_);
  cols->addWidget(right_text_);
  cols->addStretch();

  connect(left_value_, QOverload<int>::of(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
  connect(right_value_, QOverload<int>::of(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
}

ParamGetterWidget_Ratio::ValueType ParamGetterWidget_Ratio::getValue() const
{
  return { left_value_->value(), right_value_->value() };
}

bool ParamGetterWidget_Ratio::setValue(const ValueType& src)
{
  left_value_->setValue(src.first);
  right_value_->setValue(src.second);

  return true;
}

void ParamGetterWidget_Ratio::setMinimum(int minimum)
{
  left_value_->setMinimum(minimum);
  right_value_->setMinimum(minimum);
}

void ParamGetterWidget_Ratio::setMaximum(int maximum)
{
  left_value_->setMinimum(maximum);
  right_value_->setMinimum(maximum);
}

void ParamGetterWidget_Ratio::setLeftText(const QString& text)
{
  left_text_->setText(text);
}

void ParamGetterWidget_Ratio::setRightText(const QString& text)
{
  right_text_->setText(text);
}

void ParamGetterWidget_Ratio::onValueChanged()
{
  Q_EMIT valueChanged(getValue());
}
}  // namespace sa
}  // namespace gui
