#pragma once

#include <QPushButton>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>

namespace gui
{
namespace log
{
class FlightLogWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit FlightLogWidget(rclcpp::Node::SharedPtr node);

private:
  const rclcpp::Node::SharedPtr node_;
  ssh::SSHClient ssh_client_;

  QPushButton* read_button_;
  QPushButton* download_button_;
  QPushButton* clean_button_;

  qt::ListWidget* rosbag_list_;

private Q_SLOTS:
  void onReadButtonClicked();
  void onDownloadButtonClicked();
  void onCleanButtonClicked();

  void onReadFinished();
  void onDownloadFinished();
  void onCleanFinished();
};
}  // namespace log
}  // namespace gui
