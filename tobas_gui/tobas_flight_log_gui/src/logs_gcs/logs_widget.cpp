#include "tobas_flight_log_gui/logs_gcs/logs_widget.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_flight_log_gui/constants.hpp"
#include "tobas_flight_log_gui/logs_gcs/csv_export_thread.hpp"
#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace log
{
FlightLogsWidgetGCS::FlightLogsWidgetGCS(rclcpp::Node::SharedPtr node)
  : property_client_(node, "tobas_flight_log_gui/logs_gcs"), spinner_(Qt::WindowModal, this)
{
  read_button_ = new QPushButton("Read");
  clean_button_ = new QPushButton("Clean");

  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  read_button_->setEnabled(true);
  clean_button_->setEnabled(false);

  log_list_ = new tobas::qt::ListWidget();
  log_list_->setSelectionMode(QListWidget::SingleSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(new tobas::qt::Label("Ground Station", kPSize1, QFont::Bold));
  cols->addStretch();
  cols->addWidget(read_button_);
  cols->addWidget(clean_button_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(cols);
  rows->addWidget(log_list_);

  setLayout(rows);

  // Connection
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(clean_button_, &QPushButton::clicked, this, &self::onCleanButtonClicked);
  connect(log_list_, &QListWidget::itemClicked, this, &self::onListItemClicked);
}

void FlightLogsWidgetGCS::addLog(const QString& log_name)
{
  const auto list_item = new tobas::qt::ListWidgetItem();
  list_item->setSizeHint(QSize(0, kListItemHeight));
  list_item->setData(Qt::UserRole, log_name);
  log_list_->addItem(list_item);

  const auto widget = new FlightLogItemWidgetGCS(log_name);
  connect(widget, &FlightLogItemWidgetGCS::exportButtonClicked, this, &self::onExportButtonClicked);
  connect(widget, &FlightLogItemWidgetGCS::deleteButtonClicked, this, &self::onDeleteButtonClicked);
  log_list_->setItemWidget(list_item, widget);
}

void FlightLogsWidgetGCS::removeLog(const QString& log_name)
{
  const auto list_item = findLog(log_name);
  TOBAS_CHECK(list_item);
  log_list_->remove(list_item);
}

QListWidgetItem* FlightLogsWidgetGCS::findLog(const QString& log_name)
{
  for (int row = 0; row < log_list_->count(); ++row) {
    const auto list_item = log_list_->item(row);
    const auto log_widget = tobas::qt::qConstPointerCast<FlightLogItemWidgetGCS>(log_list_->itemWidget(list_item));

    if (log_widget->logName() == log_name) {
      return list_item;
    }
  }

  return nullptr;
}

void FlightLogsWidgetGCS::clearLogs()
{
  log_list_->clear();
}

void FlightLogsWidgetGCS::sortLogs()
{
  log_list_->sortItems();
}

QString FlightLogsWidgetGCS::currentLogName() const
{
  const auto cur_item = log_list_->currentItem();
  if (!cur_item) {
    qWarning() << "Log name not selected.";
    return "";
  }

  return cur_item->data(Qt::UserRole).toString();
}

void FlightLogsWidgetGCS::setCurrentLogName(const QString& log_name)
{
  for (int i = 0; i < log_list_->count(); ++i) {
    const auto item = log_list_->item(i);
    if (item->data(Qt::UserRole).toString() == log_name) {
      log_list_->setCurrentItem(item);
      return;
    }
  }

  qWarning() << log_name << "not found.";
}

void FlightLogsWidgetGCS::onReadButtonClicked()
{
  // 現在選択されているアイテムを取得
  const auto cur_text = currentLogName();

  clearLogs();

  const auto rosbag_dir = ros2::expandUser(tobas::kRosbagDirHome);
  if (!fs::is_directory(rosbag_dir)) {
    fs::create_directories(rosbag_dir);
  }

  try {
    for (const auto& entry : fs::directory_iterator(rosbag_dir)) {
      const QString log_name(entry.path().filename().c_str());
      addLog(log_name);
    }
  }
  catch (const std::exception& e) {
    tobas::qt::qErrorBox(
      this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + e.what());
    return;
  }

  if (log_list_->count() == 0) {
    tobas::qt::qWarnBox(this, "There are no flight logs saved on the ground control station.");
    return;
  }

  sortLogs();

  // 選択されていたアイテムを再び選択
  if (!cur_text.isEmpty()) {
    setCurrentLogName(cur_text);
  }

  clean_button_->setEnabled(true);
}

void FlightLogsWidgetGCS::onCleanButtonClicked()
{
  if (!tobas::qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the GCS?", tobas::qt::WARN)) {
    return;
  }

  const auto rosbag_dir = ros2::expandUser(tobas::kRosbagDirHome);
  if (!fs::is_directory(rosbag_dir)) {
    fs::create_directories(rosbag_dir);
  }

  try {
    for (const auto& entry : fs::directory_iterator(rosbag_dir)) {
      if (fs::remove_all(entry.path()) == 0) {
        tobas::qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(entry.path()));
        return;
      }
    }
  }
  catch (const std::exception& e) {
    tobas::qt::qErrorBox(
      this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + QString(e.what()));
    return;
  }

  if (log_list_->currentItem()) {
    log_list_->deselect();
    Q_EMIT logDeselected();
  }

  clearLogs();
}

void FlightLogsWidgetGCS::onExportButtonClicked(const QString& log_name)
{
  // Get the last opened directory path
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = ros2::getHomeDir();
  }

  // Set the default output file path
  auto default_out_path = fs::path(last_opened_dir) / log_name.toStdString();
  default_out_path.replace_extension(".csv");

  // Get the save file path
  const auto save_path = QFileDialog::getSaveFileName(
    this,
    "Select Output CSV File",
    QString::fromStdString(default_out_path),
    "CSV Files (*.csv)",
    nullptr,
    QFileDialog::DontUseNativeDialog);

  // Return if canceled
  if (save_path.isEmpty()) {
    return;
  }

  // Save the selected directory path
  const auto par_dir = fs::path(save_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // Export CSV file
  CsvExportThread thread(log_name, save_path);
  spinner_.start();
  const auto [success, message] = tobas::qt::startThreadAndWait(thread, &CsvExportThread::finished);
  spinner_.stop();

  // Show the result
  if (success) {
    tobas::qt::qInfoBox(this, "The flight log has been exported successfully.");
  }
  else {
    tobas::qt::qErrorBox(this, "Failed to export the flight log: " + message);
  }
}

void FlightLogsWidgetGCS::onDeleteButtonClicked(const QString& log_name)
{
  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  if (!tobas::qt::yesOrNo(this, "Do you want to delete flight log \"" + log_name + "\"?", tobas::qt::WARN)) {
    return;
  }

  if (fs::remove_all(log_path) == 0) {
    tobas::qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(log_path));
    return;
  }

  if (currentLogName() == log_name) {
    log_list_->deselect();
    Q_EMIT logDeselected();
  }

  removeLog(log_name);
}

void FlightLogsWidgetGCS::onListItemClicked(QListWidgetItem* item)
{
  const auto log_widget = tobas::qt::qConstPointerCast<FlightLogItemWidgetGCS>(log_list_->itemWidget(item));
  Q_EMIT logSelected(log_widget->logName());
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
