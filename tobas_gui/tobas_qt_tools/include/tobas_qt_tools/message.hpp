// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas
{
namespace qt
{
enum QMessageLevel
{
  INFO,
  WARN,
  ERROR,
};

void qInfoBox(QWidget* _parent, const QString& _msg);
void qWarnBox(QWidget* _parent, const QString& _msg);
void qErrorBox(QWidget* _parent, const QString& _msg);

/** Show a dialog with a Yes/No question and return true for Yes. */
bool yesOrNo(QWidget* _parent, const QString& _text, QMessageLevel _level);
}  // namespace qt
}  // namespace tobas
