#pragma once

#include <QPushButton>

#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "./load_thread.hpp"
#include "./delete_thread.hpp"
#include "./clean_thread.hpp"

namespace gui
{
namespace log
{
class FlightLogsWidgetFC : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetFC;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit FlightLogsWidgetFC(rclcpp::Node::SharedPtr node);

private:
  QPushButton* load_button_;
  QPushButton* delete_button_;
  QPushButton* clean_button_;

  LoadThreadFC read_thread_;
  DeleteThreadFC delete_thread_;
  CleanThreadFC clean_thread_;

  qt::WaitSpinnerWidget spinner_;

  qt::ListWidget* rosbag_list_;

private Q_SLOTS:
  void onReadButtonClicked();
  void onDeleteButtonClicked();
  void onCleanButtonClicked();

  void onReadFinished(bool success, const QString& message, const QStringList& rosbag_names);
  void onDeleteFinished(bool success, const QString& message);
  void onCleanFinished(bool success, const QString& message);
};
}  // namespace log
}  // namespace gui
