// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/console.hpp"

#include <algorithm>

#include <QChar>
#include <QDebug>
#include <QHeaderView>
#include <QScrollBar>

namespace tobas::gui::ctrl
{
namespace
{
constexpr int kStampCol = 0;
constexpr int kNameCol = kStampCol + 1;
constexpr int kLevelCol = kNameCol + 1;
constexpr int kMessageCol = kLevelCol + 1;
constexpr int kNumCols = kMessageCol + 1;
}  // namespace

ConsoleWidget::ConsoleWidget(const rqt::RosQtBridge& _bridge) : super(0, kNumCols)
{
  setHorizontalHeaderLabels({ "Stamp", "Name", "Level", "Message" });
  setEditTriggers(QAbstractItemView::NoEditTriggers);  // Disable editing.
  setSelectionMode(QAbstractItemView::NoSelection);    // Disable selection.
  setFocusPolicy(Qt::NoFocus);                         // Disable focus.
  setHeaderSectionsClickable(false);                   // Disable header clicks.

  const auto hor_header = horizontalHeader();
  hor_header->setSectionResizeMode(kStampCol, QHeaderView::ResizeToContents);
  hor_header->setSectionResizeMode(kNameCol, QHeaderView::ResizeToContents);
  hor_header->setSectionResizeMode(kLevelCol, QHeaderView::ResizeToContents);
  hor_header->setSectionResizeMode(kMessageCol, QHeaderView::Stretch);

  const auto ver_header = verticalHeader();
  ver_header->setVisible(false);

  connect(&_bridge, &rqt::RosQtBridge::messageReceived, this, &self::messageCb, Qt::QueuedConnection);
}

int ConsoleWidget::sizeHintForColumn(int _column) const
{
  const auto width = super::sizeHintForColumn(_column);
  if (_column != kNameCol) {
    return width;
  }

  // Fall back to the fixed cap when the proportional width is too small.
  constexpr int kNameColumnMaxWidth = 120;
  constexpr double kNameColumnWidthRatio = 0.2;
  const auto ratio_width = static_cast<int>(viewport()->width() * kNameColumnWidthRatio);
  const auto max_width = std::max(ratio_width, kNameColumnMaxWidth);
  return std::min(width, max_width);
}

void ConsoleWidget::messageCb(const tobas_msgs::msg::Message::ConstSharedPtr& _msg)
{
  // TODO: Screen messages with buttons.

  // Add a row at the top.
  insertRow(0);

  // If rows overflow, delete the oldest row.
  const auto num_rows = rowCount();
  constexpr int kMaxRows = 1000;  // Maximum number of messages to display.
  if (num_rows > kMaxRows) {
    removeRow(num_rows - 1);
  }

  const auto milliseconds = _msg->header.stamp.nanosec / 1'000'000;
  const auto stamp_text = QString("%1.%2").arg(_msg->header.stamp.sec).arg(milliseconds, 3, 10, QChar('0'));
  const auto stamp_item = new QTableWidgetItem(stamp_text);
  setItem(0, kStampCol, stamp_item);

  const auto name = QString::fromStdString(_msg->name);
  const auto name_item = new QTableWidgetItem(name);
  name_item->setToolTip(name);
  setItem(0, kNameCol, name_item);

  const auto level_item = new QTableWidgetItem();

  const auto message = QString::fromStdString(_msg->message);
  const auto message_item = new QTableWidgetItem(message);
  message_item->setToolTip(message);

  switch (_msg->level) {
    case tobas_msgs::msg::Message::LEVEL_DEBUG: {
      level_item->setText("Debug");
      constexpr auto kDebugColor = Qt::darkGreen;
      level_item->setForeground(kDebugColor);
      message_item->setForeground(kDebugColor);
      break;
    }
    case tobas_msgs::msg::Message::LEVEL_INFO: {
      level_item->setText("Info");
      constexpr auto kInfoColor = Qt::black;
      level_item->setForeground(kInfoColor);
      message_item->setForeground(kInfoColor);
      break;
    }
    case tobas_msgs::msg::Message::LEVEL_WARN: {
      level_item->setText("Warn");
      constexpr auto kWarnColor = Qt::darkYellow;
      level_item->setForeground(kWarnColor);
      message_item->setForeground(kWarnColor);
      break;
    }
    case tobas_msgs::msg::Message::LEVEL_ERROR: {
      level_item->setText("Error");
      constexpr auto kErrorColor = Qt::darkRed;
      level_item->setForeground(kErrorColor);
      message_item->setForeground(kErrorColor);
      break;
    }
    case tobas_msgs::msg::Message::LEVEL_FATAL: {
      level_item->setText("Fatal");
      constexpr auto kFatalColor = Qt::darkMagenta;
      level_item->setForeground(kFatalColor);
      message_item->setForeground(kFatalColor);
      break;
    }
    default: {
      qWarning() << "Unknown message level:" << static_cast<int>(_msg->level);
      level_item->setText("Unknown");
      constexpr auto kUnknownColor = Qt::darkGray;
      level_item->setForeground(kUnknownColor);
      message_item->setForeground(kUnknownColor);
      break;
    }
  }

  setItem(0, kLevelCol, level_item);
  setItem(0, kMessageCol, message_item);

  // Adjust the scroll position so the display position does not change before and after adding the row.
  const auto sb = verticalScrollBar();
  const auto old_scroll = sb->value();
  if (old_scroll > 0) {
    sb->setValue(old_scroll + 1);
  }
}
}  // namespace tobas::gui::ctrl
