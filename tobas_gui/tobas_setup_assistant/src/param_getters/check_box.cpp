#include "tobas_setup_assistant/param_getters/check_box.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_CheckBox::ParamGetterWidget_CheckBox(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  box_ = new QCheckBox();
  connect(box_, &QCheckBox::toggled, this, &self::onToggled);
  rows_->addWidget(box_);
}

bool ParamGetterWidget_CheckBox::getValue() const
{
  return box_->isChecked();
}

bool ParamGetterWidget_CheckBox::setValue(const bool& src)
{
  box_->setChecked(src);
  return true;
}

void ParamGetterWidget_CheckBox::setText(const QString& text)
{
  box_->setText(text);
}

void ParamGetterWidget_CheckBox::onToggled(bool checked)
{
  Q_EMIT toggled(checked);
}
}  // namespace setup_assistant
}  // namespace gui
