// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/history_line_edit.hpp"

#include <QDebug>

namespace tobas
{
namespace qt
{
void HistoryLineEdit::addHistory(const QString& text)
{
  const auto trimmed = text.trimmed();

  if (trimmed.isEmpty()) {
    qWarning() << "Cannot add an empty text to the history.";
    return;
  }

  // 直前と同じ履歴は追加しない
  if (!history_.isEmpty() && history_.last() == trimmed) {
    return;
  }

  history_.append(trimmed);

  // 上キーを押す前の状態に戻す
  history_index_ = history_.size();
}

void HistoryLineEdit::keyPressEvent(QKeyEvent* event)
{
  if (event->key() == Qt::Key_Up) {
    showPreviousHistory();
    return;
  }

  if (event->key() == Qt::Key_Down) {
    showNextHistory();
    return;
  }

  QLineEdit::keyPressEvent(event);
}

void HistoryLineEdit::showPreviousHistory()
{
  if (history_.isEmpty()) {
    return;
  }

  if (history_index_ > 0) {
    --history_index_;
  }

  setText(history_.at(history_index_));
  setCursorPosition(text().size());
}

void HistoryLineEdit::showNextHistory()
{
  if (history_.isEmpty()) {
    return;
  }

  if (history_index_ < history_.size() - 1) {
    ++history_index_;
    setText(history_.at(history_index_));
  }
  else {
    history_index_ = history_.size();
    clear();
  }

  setCursorPosition(text().size());
}
}  // namespace qt
}  // namespace tobas
