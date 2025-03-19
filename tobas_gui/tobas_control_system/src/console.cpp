#include <QVBoxLayout>
#include <QHeaderView>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_control_system/console.hpp"

namespace gui
{
namespace gcs
{
ConsoleWidget::ConsoleWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHorizontalHeaderLabels({ "Stamp", "Name", "Level", "Message" });
  table_->horizontalHeader()->setSectionResizeMode(kStampCol, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(kNameCol, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(kLevelCol, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(kMessageCol, QHeaderView::Stretch);

  const auto rows = new QVBoxLayout();
  rows->addWidget(table_);

  setLayout(rows);

  setEnabled(false);
}

void ConsoleWidget::reset()
{
  table_->removeAll();
}

void ConsoleWidget::updateNamespace(const std::string& ns)
{
  reset();

  message_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kMessageTopic), &self::messageCb, this);

  setEnabled(true);
}

void ConsoleWidget::messageCb(const tobas_std_msgs::msg::Message::ConstSharedPtr& msg)
{
  // TODO: ボタンでメッセージのスクリーニング
  // TODO: メッセージにカーソルを重ねると全文を表示 (cf. rqt_console)

  // 先頭に行を追加
  table_->insertRow(0);

  // 行が溢れていたら古い方から消す
  const auto num_rows = table_->rowCount();
  if (num_rows > kMaxRows)
    table_->removeRow(num_rows - 1);

  const auto stamp_text = QString::number(msg->header.stamp.sec) + "." + QString::number(msg->header.stamp.nanosec);
  const auto stamp_item = new QTableWidgetItem(stamp_text);
  table_->setItem(0, kStampCol, stamp_item);

  const auto name_item = new QTableWidgetItem(msg->name.c_str());
  table_->setItem(0, kNameCol, name_item);

  const auto level_item = new QTableWidgetItem();
  const auto message_item = new QTableWidgetItem(msg->message.c_str());

  switch (msg->level)
  {
    case tobas_std_msgs::msg::Message::LEVEL_DEBUG:
      level_item->setText("Debug");
      level_item->setForeground(kDebugColor);
      message_item->setForeground(kDebugColor);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_INFO:
      level_item->setText("Info");
      level_item->setForeground(kInfoColor);
      message_item->setForeground(kInfoColor);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_WARN:
      level_item->setText("Warn");
      level_item->setForeground(kWarnColor);
      message_item->setForeground(kWarnColor);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_ERROR:
      level_item->setText("Error");
      level_item->setForeground(kErrorColor);
      message_item->setForeground(kErrorColor);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_FATAL:
      level_item->setText("Fatal");
      level_item->setForeground(kFatalColor);
      message_item->setForeground(kFatalColor);
      break;
    default:
      RCLCPP_WARN_STREAM(node_->get_logger(), "Unknown message level: " << static_cast<int>(msg->level));
      level_item->setText("Unknown");
      level_item->setForeground(kUnknownColor);
      message_item->setForeground(kUnknownColor);
      break;
  }

  table_->setItem(0, kLevelCol, level_item);
  table_->setItem(0, kMessageCol, message_item);
}
}  // namespace gcs
}  // namespace gui
