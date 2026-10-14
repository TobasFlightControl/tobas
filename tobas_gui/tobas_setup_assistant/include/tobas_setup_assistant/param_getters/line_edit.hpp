// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>

#include "./base.hpp"

namespace tobas::gui::sa
{
class ParamGetterWidget_LineEdit : public ParamGetterWidget<QString>
{
  Q_OBJECT

  using self = ParamGetterWidget_LineEdit;
  using super = ParamGetterWidget<QString>;

Q_SIGNALS:
  void textChanged(const QString& _text);

public:
  explicit ParamGetterWidget_LineEdit(const QString& _param_name, const QString& _description_text = "");

  QString getValue() const override;
  bool setValue(const QString& _src) override;

private Q_SLOTS:
  void onTextChanged(const QString& _text);

private:
  QLineEdit* line_;
};
}  // namespace tobas::gui::sa
