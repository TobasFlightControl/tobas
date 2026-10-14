// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/password_edit.hpp"

#include <QAction>

#include "tobas_qt_tools/path.hpp"

namespace tobas
{
namespace qt
{
PasswordEdit::PasswordEdit(QWidget* _parent) : super(_parent)
{
  const auto rsrc_path = getResourcePath();
  eye_on_ = QIcon(rsrc_path + "/eye_on.png");
  eye_off_ = QIcon(rsrc_path + "/eye_off.png");

  toggle_ = addAction(eye_off_, QLineEdit::TrailingPosition);
  toggle_->setCheckable(true);

  hideText();

  connect(toggle_, &QAction::toggled, this, &self::onIconToggled);
}

void PasswordEdit::showText()
{
  setMode(true);
}

void PasswordEdit::hideText()
{
  setMode(false);
}

void PasswordEdit::reset()
{
  clear();
  hideText();
}

void PasswordEdit::setMode(bool _on)
{
  setEchoMode(_on ? QLineEdit::Normal : QLineEdit::Password);
  toggle_->setIcon(_on ? eye_on_ : eye_off_);
  toggle_->setToolTip(_on ? "Hide password" : "Show password");
}

void PasswordEdit::onIconToggled(bool _on)
{
  const auto cursor_pos = cursorPosition();  // Keep the cursor position.
  setMode(_on);
  setCursorPosition(cursor_pos);
}
}  // namespace qt
}  // namespace tobas
