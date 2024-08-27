#include "tobas_setup_assistant/param_getters/int_range.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_IntRange::ParamGetterWidget_IntRange(
  const QString& param_name,
  const QString& description_text,
  int minimum,
  int maximum,
  int single_step,
  const std::pair<int, int>& _default,
  const QString& suffix)
  : super(param_name, description_text)
{
  auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  min_ = new IntGetter("min", minimum, maximum, single_step, _default.first, suffix);
  cols->addWidget(min_);

  max_ = new IntGetter("max", minimum, maximum, single_step, _default.second, suffix);
  cols->addWidget(max_);

  connect(min_, &IntGetter::valueChanged, this, &self::onValueChanged);
  connect(max_, &IntGetter::valueChanged, this, &self::onValueChanged);
}

std::pair<int, int> ParamGetterWidget_IntRange::get() const
{
  return { min(), max() };
}

bool ParamGetterWidget_IntRange::set(const std::pair<int, int>& src)
{
  min_->set(src.first);
  max_->set(src.second);
  return true;
}

int ParamGetterWidget_IntRange::min() const
{
  return min_->get();
}

int ParamGetterWidget_IntRange::max() const
{
  return max_->get();
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
