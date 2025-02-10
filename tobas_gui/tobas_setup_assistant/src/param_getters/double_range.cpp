#include "tobas_setup_assistant/param_getters/double_range.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_DoubleRange::ParamGetterWidget_DoubleRange(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  min_ = new DoubleGetter("min");
  cols->addWidget(min_);

  max_ = new DoubleGetter("max");
  cols->addWidget(max_);

  connect(min_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
  connect(max_, &DoubleGetter::valueChanged, this, &self::onValueChanged);
}

std::pair<double, double> ParamGetterWidget_DoubleRange::getValue() const
{
  return { min(), max() };
}

bool ParamGetterWidget_DoubleRange::setValue(const std::pair<double, double>& src)
{
  min_->setValue(src.first);
  max_->setValue(src.second);
  return true;
}

void ParamGetterWidget_DoubleRange::setDecimals(int decimals)
{
  min_->setDecimals(decimals);
  max_->setDecimals(decimals);
}

void ParamGetterWidget_DoubleRange::setMinimum(double minimum)
{
  min_->setMinimum(minimum);
  max_->setMinimum(minimum);
}

void ParamGetterWidget_DoubleRange::setMaximum(double maximum)
{
  min_->setMaximum(maximum);
  max_->setMaximum(maximum);
}

void ParamGetterWidget_DoubleRange::setSingleStep(double single_step)
{
  min_->setSingleStep(single_step);
  max_->setSingleStep(single_step);
}

void ParamGetterWidget_DoubleRange::setSuffix(const QString& suffix)
{
  min_->setSuffix(suffix);
  max_->setSuffix(suffix);
}

double ParamGetterWidget_DoubleRange::min() const
{
  return min_->getValue();
}

double ParamGetterWidget_DoubleRange::max() const
{
  return max_->getValue();
}

bool ParamGetterWidget_DoubleRange::isValid() const
{
  return min() <= max();
}

void ParamGetterWidget_DoubleRange::onValueChanged(double)
{
  Q_EMIT valueChanged({ min(), max() });
}
}  // namespace sa
}  // namespace gui
