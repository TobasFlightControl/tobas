#include "tobas_setup_assistant/param_getters/line_edit.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_LineEdit::ParamGetterWidget_LineEdit(
  const QString& param_name,
  const QString& description_text,
  const QString& _default)
  : super(param_name, description_text)
{
  line_ = new QLineEdit(_default);
  rows_->addWidget(line_);

  connect(line_, &QLineEdit::textChanged, this, &self::onTextChanged);
}

QString ParamGetterWidget_LineEdit::get() const
{
  return line_->text();
}

bool ParamGetterWidget_LineEdit::set(const QString& src)
{
  line_->setText(src);
  return true;
}

void ParamGetterWidget_LineEdit::onTextChanged(const QString& text)
{
  Q_EMIT textChanged(text);
}
}  // namespace setup_assistant
}  // namespace gui
