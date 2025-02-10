#include "tobas_setup_assistant/param_getters/line_edit.hpp"

namespace gui
{
namespace sa
{
ParamGetterWidget_LineEdit::ParamGetterWidget_LineEdit(const QString& param_name, const QString& description_text)
  : super(param_name, description_text)
{
  line_ = new QLineEdit();
  rows_->addWidget(line_);

  connect(line_, &QLineEdit::textChanged, this, &self::onTextChanged);
}

QString ParamGetterWidget_LineEdit::getValue() const
{
  return line_->text();
}

bool ParamGetterWidget_LineEdit::setValue(const QString& src)
{
  line_->setText(src);
  return true;
}

void ParamGetterWidget_LineEdit::onTextChanged(const QString& text)
{
  Q_EMIT textChanged(text);
}
}  // namespace sa
}  // namespace gui
