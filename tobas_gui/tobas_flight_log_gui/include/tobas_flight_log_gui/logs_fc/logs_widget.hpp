#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ssh_client/ssh_client.hpp>

#include "./clean_thread.hpp"
#include "./delete_thread.hpp"
#include "./download_thread.hpp"
#include "./read_thread.hpp"

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
  static constexpr int kListItemHeight = 40;

Q_SIGNALS:
  void logDownloaded(const QString& log_name);

public:
  explicit FlightLogsWidgetFC(rclcpp::Node::SharedPtr node);

  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();
  void sortLogs();

private:
  QPushButton* read_button_;
  QPushButton* clean_button_;

  ReadThread read_thread_;
  CleanThread clean_thread_;
  DownloadThread download_thread_;
  DeleteThread delete_thread_;

  qt::WaitSpinnerWidget spinner_;

  qt::ListWidget* log_list_;

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onDownloadButtonClicked(const QString& log_name);
  void onDeleteButtonClicked(const QString& log_name);

  void onReadFinished(bool success, const QString& message, const QStringList& log_names);
  void onCleanFinished(bool success, const QString& message);
  void onDownloadFinished(bool success, const QString& message);
  void onDeleteFinished(bool success, const QString& message);
};
}  // namespace log
}  // namespace gui
