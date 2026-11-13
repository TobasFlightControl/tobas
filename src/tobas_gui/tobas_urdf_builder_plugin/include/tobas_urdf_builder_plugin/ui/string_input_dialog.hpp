// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDialog>
#include <QDialogButtonBox>
#include <QLabel>
#include <QLineEdit>

namespace tobas::gui::ub::ui
{
class StringInputDialog : public QDialog
{
  Q_OBJECT

  using self = StringInputDialog;
  using super = QDialog;

public:
  explicit StringInputDialog(
    QWidget* _parent,
    const QString& _title,
    const QString& _name,
    const QString& _default_text = "",
    const QStringList& _excludeds = QStringList());

  QString getText() const;

private Q_SLOTS:
  void onLineEditTextChanged(const QString& _text);

private:
  const QStringList& excludeds_;

  QLineEdit* line_edit_;
  QLabel* warn_label_;
  QDialogButtonBox* button_box_;

  void enableOkButton(bool _enable);
};
}  // namespace tobas::gui::ub::ui
