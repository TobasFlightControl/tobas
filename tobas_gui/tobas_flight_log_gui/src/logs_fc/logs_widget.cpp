#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/cast.hpp>

#include "tobas_flight_log_gui/logs_fc/logs_widget.hpp"
#include "tobas_flight_log_gui/logs_fc/log_item.hpp"
#include "tobas_flight_log_gui/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogsWidgetFC::FlightLogsWidgetFC(rclcpp::Node::SharedPtr node)
  : read_thread_(node)
  , clean_thread_(node)
  , download_thread_(node)
  , delete_thread_(node)
  , spinner_(Qt::WindowModal, this)
{
  read_button_ = new QPushButton("Read");
  clean_button_ = new QPushButton("Clean");

  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  read_button_->setEnabled(true);
  clean_button_->setEnabled(false);

  log_list_ = new qt::ListWidget();
  log_list_->setSelectionMode(QListWidget::NoSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(read_button_);
  cols->addWidget(clean_button_);
  cols->addStretch();

  const auto rows = new QVBoxLayout();
  qt::addWidgetCenter(new qt::Label("Flight Controller", kPSize1, QFont::Bold), rows);
  rows->addLayout(cols);
  rows->addWidget(log_list_);

  setLayout(rows);

  // Connection
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(clean_button_, &QPushButton::clicked, this, &self::onCleanButtonClicked);
  connect(&read_thread_, &ReadThread::finished, this, &self::onReadFinished);
  connect(&clean_thread_, &CleanThread::finished, this, &self::onCleanFinished);
  connect(&download_thread_, &DownloadThread::finished, this, &self::onDownloadFinished);
  connect(&delete_thread_, &DeleteThread::finished, this, &self::onDeleteFinished);
}

void FlightLogsWidgetFC::addLog(const QString& log_name)
{
  const auto list_item = new qt::ListWidgetItem();
  list_item->setSizeHint(QSize(0, kListItemHeight));
  list_item->setData(Qt::UserRole, log_name);
  log_list_->addItem(list_item);

  const auto widget = new FlightLogItemWidgetFC(log_name);
  connect(widget, &FlightLogItemWidgetFC::downloadButtonClicked, this, &self::onDownloadButtonClicked);
  connect(widget, &FlightLogItemWidgetFC::deleteButtonClicked, this, &self::onDeleteButtonClicked);
  log_list_->setItemWidget(list_item, widget);
}

void FlightLogsWidgetFC::removeLog(const QString& log_name)
{
  const auto list_item = findLog(log_name);
  TOBAS_CHECK(list_item);
  log_list_->remove(list_item);
}

QListWidgetItem* FlightLogsWidgetFC::findLog(const QString& log_name)
{
  for (int row = 0; row < log_list_->count(); ++row) {
    const auto list_item = log_list_->item(row);
    const auto log_widget = qt::qConstPointerCast<FlightLogItemWidgetFC>(log_list_->itemWidget(list_item));

    if (log_widget->logName() == log_name) {
      return list_item;
    }
  }

  return nullptr;
}

void FlightLogsWidgetFC::clearLogs()
{
  log_list_->clear();
}

void FlightLogsWidgetFC::sortLogs()
{
  log_list_->sortItems();
}

void FlightLogsWidgetFC::onReadButtonClicked()
{
  read_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogsWidgetFC::onCleanButtonClicked()
{
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the FC?", qt::QMessageLevel::WARN)) {
    return;
  }

  clean_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogsWidgetFC::onDownloadButtonClicked(const QString& log_name)
{
  const auto rosbag_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  if (fs::exists(rosbag_path)) {
    if (qt::yesOrNo(
          this,
          QString(rosbag_path.c_str()) + " already exists. Do you want to overwrite it?",
          qt::QMessageLevel::WARN)) {
      fs::remove_all(rosbag_path);
    }
    else {
      return;
    }
  }

  download_thread_.setLogName(log_name);
  download_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogsWidgetFC::onDeleteButtonClicked(const QString& log_name)
{
  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  if (!qt::yesOrNo(this, "Do you want to delete flight log \"" + log_name + "\"?", qt::QMessageLevel::WARN)) {
    return;
  }

  delete_thread_.setLogName(log_name);
  delete_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogsWidgetFC::onReadFinished(bool success, const QString& message, const QStringList& log_names)
{
  spinner_.hide();
  spinner_.stop();

  if (!success) {
    qt::qErrorBox(this, message);
    return;
  }

  clearLogs();

  if (log_names.size() == 0) {
    qt::qWarnBox(this, "There are no flight logs saved on the flight controller.");
    return;
  }

  for (const auto& log_name : log_names) {
    addLog(log_name);
  }

  sortLogs();

  clean_button_->setEnabled(true);
}

void FlightLogsWidgetFC::onCleanFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success) {
    qt::qErrorBox(this, message);
    return;
  }

  clearLogs();
}

void FlightLogsWidgetFC::onDownloadFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success) {
    qt::qErrorBox(this, message);
    return;
  }

  Q_EMIT logDownloaded(download_thread_.getLogName());
}

void FlightLogsWidgetFC::onDeleteFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success) {
    qt::qErrorBox(this, message);
    return;
  }

  removeLog(delete_thread_.getLogName());
}
}  // namespace log
}  // namespace gui
