#include "tobas_setup_assistant/param_getters/int_range.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_IntRange::ParamGetterWidget_IntRange(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  min_ = new IntGetter("min");
  cols->addWidget(min_);

  max_ = new IntGetter("max");
  cols->addWidget(max_);

  connect(min_, &IntGetter::valueChanged, this, &self::onValueChanged);
  connect(max_, &IntGetter::valueChanged, this, &self::onValueChanged);
}

std::pair<int, int> ParamGetterWidget_IntRange::getValue() const
{
  return { min(), max() };
}

bool ParamGetterWidget_IntRange::setValue(const std::pair<int, int>& src)
{
  min_->setValue(src.first);
  max_->setValue(src.second);
  return true;
}

void ParamGetterWidget_IntRange::setMinimum(int minimum)
{
  min_->setMinimum(minimum);
  max_->setMinimum(minimum);
}

void ParamGetterWidget_IntRange::setMaximum(int maximum)
{
  min_->setMaximum(maximum);
  max_->setMaximum(maximum);
}

void ParamGetterWidget_IntRange::setSingleStep(int single_step)
{
  min_->setSingleStep(single_step);
  max_->setSingleStep(single_step);
}

void ParamGetterWidget_IntRange::setSuffix(const QString& suffix)
{
  min_->setSuffix(suffix);
  max_->setSuffix(suffix);
}

int ParamGetterWidget_IntRange::min() const
{
  return min_->getValue();
}

int ParamGetterWidget_IntRange::max() const
{
  return max_->getValue();
}

bool ParamGetterWidget_IntRange::isValid() const
{
  return min() <= max();
}

void ParamGetterWidget_IntRange::onValueChanged(int)
{
  Q_EMIT valueChanged({ min(), max() });
}
}  // namespace setup_assistant
}  // namespace gui
