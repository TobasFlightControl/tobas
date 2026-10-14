// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QIcon>
#include <QLineEdit>

namespace tobas::qt
{
class PasswordEdit : public QLineEdit
{
  Q_OBJECT

  using self = PasswordEdit;
  using super = QLineEdit;

public:
  explicit PasswordEdit(QWidget* _parent = nullptr);

  void showText();
  void hideText();

  /** Return to the startup state, clear plus hide. */
  void reset();

private:
  QIcon eye_on_;
  QIcon eye_off_;

  QAction* toggle_;

  void setMode(bool _on);

private Q_SLOTS:
  void onIconToggled(bool _on);
};
}  // namespace tobas::qt
