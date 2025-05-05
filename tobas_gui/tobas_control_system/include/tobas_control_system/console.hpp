#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

#include <tobas_std_msgs/msg/message.hpp>

namespace gui
{
namespace gcs
{
class ConsoleWidget : public QWidget
{
  Q_OBJECT

  using self = ConsoleWidget;
  using super = QWidget;

  static constexpr int kMaxRows = 1000;  // 表示するメッセージの最大数

  static constexpr int kStampCol = 0;
  static constexpr int kNameCol = 1;
  static constexpr int kLevelCol = 2;
  static constexpr int kMessageCol = 3;
  static constexpr int kNumCols = 4;

  // メッセージの色 (ケース不問)
  static constexpr auto kDebugColor = Qt::darkGreen;
  static constexpr auto kInfoColor = Qt::black;
  static constexpr auto kWarnColor = Qt::darkYellow;
  static constexpr auto kErrorColor = Qt::darkRed;
  static constexpr auto kFatalColor = Qt::darkMagenta;
  static constexpr auto kUnknownColor = Qt::darkGray;

public:
  explicit ConsoleWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  std::string ns_;

  qt::TableWidget* table_;

  ros2::SubscriberPtr<tobas_std_msgs::msg::Message> message_sub_;

  void messageCb(const tobas_std_msgs::msg::Message::ConstSharedPtr& msg);
};
}  // namespace gcs
}  // namespace gui
