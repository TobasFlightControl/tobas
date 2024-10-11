#pragma once

#include <rclcpp/node.hpp>
#include <QLineEdit>
#include <QPushButton>

namespace gui
{
namespace log
{
class FlightLogRecorderWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogRecorderWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit FlightLogRecorderWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  std::string ns_;

  QLineEdit* rosbag_name_;
  QPushButton* start_button_;
  QPushButton* stop_button_;

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
};
}  // namespace log
}  // namespace gui
