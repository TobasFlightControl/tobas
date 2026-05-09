// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QKeyEvent>
#include <QLineEdit>
#include <QStringList>

namespace tobas
{
namespace qt
{
class HistoryLineEdit : public QLineEdit
{
  Q_OBJECT

public:
  using QLineEdit::QLineEdit;

  void addHistory(const QString& text);

protected:
  void keyPressEvent(QKeyEvent* event) override;

private:
  void showPreviousHistory();
  void showNextHistory();

private:
  QStringList history_;
  int history_index_ = 0;
};
}  // namespace qt
}  // namespace tobas
