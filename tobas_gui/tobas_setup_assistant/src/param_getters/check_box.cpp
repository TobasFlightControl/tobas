#include "tobas_setup_assistant/param_getters/check_box.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_CheckBox::ParamGetterWidget_CheckBox(
  const QString& param_name,
  const QString& description_text,
  const QString& check_box_text,
  bool _default)
  : super(param_name, description_text)
{
  box_ = new QCheckBox(check_box_text);
  box_->setChecked(_default);
  connect(box_, &QCheckBox::toggled, this, &self::onToggled);
  rows_->addWidget(box_);
}

bool ParamGetterWidget_CheckBox::get() const
{
  return box_->isChecked();
}

bool ParamGetterWidget_CheckBox::set(const bool& src)
{
  box_->setChecked(src);
  return true;
}

void ParamGetterWidget_CheckBox::onToggled(bool checked)
{
  Q_EMIT toggled(checked);
}
}  // namespace setup_assistant
}  // namespace gui
