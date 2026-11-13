// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/line_edit.hpp"

namespace tobas::gui::sa
{
ParamGetterWidget_LineEdit::ParamGetterWidget_LineEdit(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  line_ = new QLineEdit();
  rows_->addWidget(line_);

  connect(line_, &QLineEdit::textChanged, this, &self::onTextChanged);
}

QString ParamGetterWidget_LineEdit::getValue() const
{
  return line_->text();
}

bool ParamGetterWidget_LineEdit::setValue(const QString& _src)
{
  line_->setText(_src);
  return true;
}

void ParamGetterWidget_LineEdit::onTextChanged(const QString& _text)
{
  Q_EMIT textChanged(_text);
}
}  // namespace tobas::gui::sa
