#pragma once

#include <QPushButton>

#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "./read_thread.hpp"
#include "./download_thread.hpp"
#include "./clean_thread.hpp"

namespace gui
{
namespace log
{
class FlightLogReaderWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogReaderWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit FlightLogReaderWidget(rclcpp::Node::SharedPtr node);

private:
  QPushButton* read_button_;
  QPushButton* download_button_;
  QPushButton* clean_button_;

  ReadThread read_thread_;
  DownloadThread download_thread_;
  CleanThread clean_thread_;

  qt::WaitSpinnerWidget spinner_;

  qt::ListWidget* rosbag_list_;

private Q_SLOTS:
  void onReadButtonClicked();
  void onDownloadButtonClicked();
  void onCleanButtonClicked();

  void onReadFinished(bool success, const QString& message, const QStringList& rosbag_names);
  void onDownloadFinished(bool success, const QString& message);
  void onCleanFinished(bool success, const QString& message);
};
}  // namespace log
}  // namespace gui
