// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/message.hpp"

#include <QMessageBox>

namespace tobas
{
namespace qt
{
void qInfoBox(QWidget* parent, const QString& msg)
{
  QMessageBox::information(parent, "INFO", msg);
}

void qWarnBox(QWidget* parent, const QString& msg)
{
  QMessageBox::warning(parent, "WARN", msg);
}

void qErrorBox(QWidget* parent, const QString& msg)
{
  QMessageBox::critical(parent, "ERROR", msg);
}

bool yesOrNo(QWidget* parent, const QString& text, QMessageLevel level)
{
  QMessageBox msg_box(parent);

  // Set the message level.
  switch (level) {
    case QMessageLevel::INFO:
      msg_box.setIcon(QMessageBox::Icon::Information);
      msg_box.setWindowTitle("INFO");
      break;
    case QMessageLevel::WARN:
      msg_box.setIcon(QMessageBox::Icon::Warning);
      msg_box.setWindowTitle("WARN");
      break;
    case QMessageLevel::ERROR:
      msg_box.setIcon(QMessageBox::Icon::Critical);
      msg_box.setWindowTitle("ERROR");
      break;
    default:
      throw;
  }

  // Set text.
  msg_box.setText(text);

  // Set text.
  msg_box.setText(text);

  // Set buttons.
  // Layout is determined automatically. There is no clear rule, but consistent rules across the app are important:
  // https://nanika.design/blog/1162/
  msg_box.setStandardButtons(QMessageBox::StandardButton::Yes | QMessageBox::StandardButton::No);
  msg_box.setDefaultButton(QMessageBox::StandardButton::No);

  // Get the user response and return true for Yes.
  return msg_box.exec() == QMessageBox::StandardButton::Yes;
}
}  // namespace qt
}  // namespace tobas
