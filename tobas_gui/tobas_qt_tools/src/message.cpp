// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/message.hpp"

#include <QDebug>
#include <QMessageBox>

namespace tobas::qt
{
void qInfoBox(QWidget* _parent, const QString& _msg)
{
  qInfo().noquote() << _msg;
  QMessageBox::information(_parent, "INFO", _msg);
}

void qWarnBox(QWidget* _parent, const QString& _msg)
{
  qWarning().noquote() << _msg;
  QMessageBox::warning(_parent, "WARN", _msg);
}

void qErrorBox(QWidget* _parent, const QString& _msg)
{
  qCritical().noquote() << _msg;
  QMessageBox::critical(_parent, "ERROR", _msg);
}

bool yesOrNo(QWidget* _parent, const QString& _text, QMessageLevel _level)
{
  QMessageBox msg_box(_parent);

  // Set the message level.
  switch (_level) {
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
  msg_box.setText(_text);

  // Set text.
  msg_box.setText(_text);

  // Set buttons.
  // Layout is determined automatically. There is no clear rule, but consistent rules across the app are important:
  // Ref: https://nanika.design/blog/1162/
  msg_box.setStandardButtons(QMessageBox::StandardButton::Yes | QMessageBox::StandardButton::No);
  msg_box.setDefaultButton(QMessageBox::StandardButton::No);

  // Get the user response and return true for Yes.
  return msg_box.exec() == QMessageBox::StandardButton::Yes;
}
}  // namespace tobas::qt
