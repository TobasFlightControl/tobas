#include "tobas_control_system/console.hpp"

#include <QDebug>
#include <QHeaderView>
#include <QScrollBar>
#include <QVBoxLayout>

namespace gui
{
namespace ctrl
{
ConsoleWidget::ConsoleWidget(const RosQtBridge& bridge)
{
  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHorizontalHeaderLabels({ "Stamp", "Name", "Level", "Message" });
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);  // 編集禁止
  table_->setSelectionMode(QAbstractItemView::NoSelection);    // 選択禁止
  table_->setFocusPolicy(Qt::NoFocus);                         // フォーカス禁止
  table_->setHeaderSectionsClickable(false);                   // ヘッダのクリック禁止

  const auto hor_header = table_->horizontalHeader();
  hor_header->setSectionResizeMode(kStampCol, QHeaderView::ResizeToContents);
  hor_header->setSectionResizeMode(kNameCol, QHeaderView::ResizeToContents);
  hor_header->setSectionResizeMode(kLevelCol, QHeaderView::ResizeToContents);
  hor_header->setSectionResizeMode(kMessageCol, QHeaderView::Stretch);

  const auto ver_header = table_->verticalHeader();
  ver_header->setVisible(false);

  const auto rows = new QVBoxLayout();
  rows->addWidget(table_);
  setLayout(rows);

  connect(&bridge, &RosQtBridge::messageReceived, this, &self::messageCb, Qt::QueuedConnection);
}

void ConsoleWidget::reset()
{
  table_->removeAll();
}

void ConsoleWidget::messageCb(const tobas_msgs::msg::Message::ConstSharedPtr& msg)
{
  // TODO: ボタンでメッセージのスクリーニング

  // 先頭に行を追加
  table_->insertRow(0);

  // 行が溢れていたら古い方から消す
  const auto num_rows = table_->rowCount();
  if (num_rows > kMaxRows) {
    table_->removeRow(num_rows - 1);
  }

  const auto stamp_text = QString::number(msg->header.stamp.sec) + "." + QString::number(msg->header.stamp.nanosec);
  const auto stamp_item = new QTableWidgetItem(stamp_text);
  table_->setItem(0, kStampCol, stamp_item);

  const auto name = QString::fromStdString(msg->name);
  const auto name_item = new QTableWidgetItem(name);
  table_->setItem(0, kNameCol, name_item);

  const auto level_item = new QTableWidgetItem();

  const auto message = QString::fromStdString(msg->message);
  const auto message_item = new QTableWidgetItem(message);
  message_item->setToolTip(message);

  switch (msg->level) {
    case tobas_msgs::msg::Message::LEVEL_DEBUG:
      level_item->setText("Debug");
      level_item->setForeground(kDebugColor);
      message_item->setForeground(kDebugColor);
      break;
    case tobas_msgs::msg::Message::LEVEL_INFO:
      level_item->setText("Info");
      level_item->setForeground(kInfoColor);
      message_item->setForeground(kInfoColor);
      break;
    case tobas_msgs::msg::Message::LEVEL_WARN:
      level_item->setText("Warn");
      level_item->setForeground(kWarnColor);
      message_item->setForeground(kWarnColor);
      break;
    case tobas_msgs::msg::Message::LEVEL_ERROR:
      level_item->setText("Error");
      level_item->setForeground(kErrorColor);
      message_item->setForeground(kErrorColor);
      break;
    case tobas_msgs::msg::Message::LEVEL_FATAL:
      level_item->setText("Fatal");
      level_item->setForeground(kFatalColor);
      message_item->setForeground(kFatalColor);
      break;
    default:
      qWarning() << "Unknown message level:" << static_cast<int>(msg->level);
      level_item->setText("Unknown");
      level_item->setForeground(kUnknownColor);
      message_item->setForeground(kUnknownColor);
      break;
  }

  table_->setItem(0, kLevelCol, level_item);
  table_->setItem(0, kMessageCol, message_item);

  // 行の追加前後で表示位置が変化しないようにスクロール位置を調整
  const auto sb = table_->verticalScrollBar();
  const auto old_scroll = sb->value();
  if (old_scroll > 0) {
    sb->setValue(old_scroll + 1);
  }
}
}  // namespace ctrl
}  // namespace gui
