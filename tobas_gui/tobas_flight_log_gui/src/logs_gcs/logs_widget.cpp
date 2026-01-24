#include "tobas_flight_log_gui/logs_gcs/logs_widget.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rosbag2_cpp/reindexer.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_flight_log_gui/constants.hpp"
#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"

#include <rosbag2_storage/storage_filter.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
void CsvExportWorker::process(const QString& log_name, const QString& savePath)
{
  try {
    std::ofstream csvFile(savePath.toStdString());
    if (!csvFile.is_open()) {
      std::cerr << "Couldn't write to csv file: " << savePath.toStdString() << std::endl;
      return;
    }
    csvFile << CsvExportWorker::exportCsvHeader;

    const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

    if (!open(log_path.string())) {
      if (!reindex(log_path.string())) {
        return;
      }
      if (!open(log_path.string())) {
        return;
      }
    }

    while (reader_.has_next()) {
      const auto msg = reader_.read_next();
      const auto& cur_time = msg->recv_timestamp;  // [ns]

      rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

      if (msg->topic_name.ends_with(path::join("/", tobas::kImuRawTopic))) {
        curData_.cur_imu = std::make_shared<tobas_msgs::msg::Imu>(imu_decoder_.decode(cur_time, ser_msg));
        csvFile << makeCsvRow(cur_time);
      }
      else {
        if (msg->topic_name.ends_with(path::join("/", tobas::kOdometryTopic))) {
          curData_.cur_odom = std::make_shared<tobas_msgs::msg::Odometry>(odom_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kImuFiltTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kMagTopic))) {
          curData_.cur_mag = std::make_shared<tobas_msgs::msg::MagneticField>(mag_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kGnssTopic))) {
          curData_.cur_gnss = std::make_shared<tobas_msgs::msg::Gnss>(gnss_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kRcInputTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kBatteryTopic))) {
          curData_.cur_battery = std::make_shared<tobas_msgs::msg::Battery>(battery_decoder_.decode(cur_time, ser_msg));
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kCpuTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorStatesTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorSpeedsCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointStatesTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointPosCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointVelCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kJointEffCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kIcePropulsionSystemCmdTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kImuSamplingTimeTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kControlLatencyTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kVibrationLevelTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kDisturbanceForceTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kObsvFeedbackTopic))) {
        }
        else if (msg->topic_name.ends_with(path::join("/", tobas::kMRCtrlFeedbackTopic))) {
        }
      }
    }
    csvFile.close();
  }
  catch (const std::exception& e) {
    Q_EMIT error(QString::fromStdString(e.what()));
    return;
  }
  Q_EMIT finished();
}

// TODO rosbag.hpp
bool CsvExportWorker::reindex(const std::string& rosbag_path)
{
  rosbag2_cpp::Reindexer reindexer;

  rosbag2_storage::StorageOptions options;
  options.uri = rosbag_path;
  options.storage_id = "mcap";

  try {
    reindexer.reindex(options);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to reindex " << QString::fromStdString(rosbag_path) + ": " << e.what();
    return false;
  }

  return true;
}

// TODO rosbag.hpp
bool CsvExportWorker::open(const std::string& rosbag_path)
{
  try {
    reader_.open(rosbag_path);
  }
  catch (const std::exception& e) {
    qWarning() << "Failed to open " << QString::fromStdString(rosbag_path) + ": " << e.what();
    return false;
  }

  return true;
}

template <typename T, typename Func>
std::string CsvExportWorker::getLogString(
  const std::shared_ptr<T>& cur_ptr,
  std::shared_ptr<T>& last_ptr,
  const std::string& empty_str,
  Func formatter)
{
  if (cur_ptr != nullptr && cur_ptr != last_ptr) {
    last_ptr = cur_ptr;
    return formatter(cur_ptr);
  }
  return empty_str;
}

std::string CsvExportWorker::makeCsvRow(const auto& cur_time)
{
  std::string csv_line = std::to_string(cur_time / 1000000000.0) + ",";

  // Pose
  // csv_line += sgetLogString(curData_.cur_odom, lastData_.last_odom, ",,,,,,",
  //   [](const auto& msg)
  //   {
  //     return std::to_string(msg->latitude) + "," ;
  // });

  // Twist

  // Accel

  // Imu
  csv_line += std::to_string(curData_.cur_imu->accel.x) + "," + std::to_string(curData_.cur_imu->accel.y) + "," +
              std::to_string(curData_.cur_imu->accel.z) + "," + std::to_string(curData_.cur_imu->gyro.x) + "," +
              std::to_string(curData_.cur_imu->gyro.y) + "," + std::to_string(curData_.cur_imu->gyro.z) + "," +
              std::to_string(curData_.cur_imu->dgyro.x) + "," + std::to_string(curData_.cur_imu->dgyro.y) + "," +
              std::to_string(curData_.cur_imu->dgyro.z) + ",";

  // Magnetic_Field
  csv_line += getLogString(
    curData_.cur_mag,
    lastData_.last_mag,
    ",,,",
    [](const auto& msg)
    { return std::to_string(msg->mag.x) + "," + std::to_string(msg->mag.y) + "," + std::to_string(msg->mag.z) + ","; });

  // Gnss
  csv_line += getLogString(
    curData_.cur_gnss,
    lastData_.last_gnss,
    ",,,,,,",
    [](const auto& msg)
    {
      return std::to_string(msg->latitude) + "," + std::to_string(msg->longitude) + "," +
             std::to_string(msg->altitude) + "," + std::to_string(msg->ground_speed.x) + "," +
             std::to_string(msg->ground_speed.y) + "," + std::to_string(msg->ground_speed.z) + ",";
    });

  // RC Input

  // Battery
  csv_line += getLogString(
    curData_.cur_battery,
    lastData_.last_battery,
    ",",
    [](const auto& msg) { return std::to_string(msg->voltage) + "," + std::to_string(msg->current); });

  // Engine

  // CPU

  // Rotor Speed

  // Rotor Link

  // Latency

  // Vibration_Level

  // Disturbance_Force

  // Observer

  // Multirotor_Controller

  return csv_line + "\n";
}

FlightLogsWidgetGCS::FlightLogsWidgetGCS() : spinner_(Qt::WindowModal, this)
{
  read_button_ = new QPushButton("Read");
  clean_button_ = new QPushButton("Clean");

  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  read_button_->setEnabled(true);
  clean_button_->setEnabled(false);

  log_list_ = new qt::ListWidget();
  log_list_->setSelectionMode(QListWidget::SingleSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(new qt::Label("Ground Station", kPSize1, QFont::Bold));
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
  const auto list_item = new qt::ListWidgetItem();
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
    const auto log_widget = qt::qConstPointerCast<FlightLogItemWidgetGCS>(log_list_->itemWidget(list_item));

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

  qWarning() << log_name << " not found.";
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
    qt::qErrorBox(this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + e.what());
    return;
  }

  if (log_list_->count() == 0) {
    qt::qWarnBox(this, "There are no flight logs saved on the ground control station.");
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
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the GCS?", qt::WARN)) {
    return;
  }

  const auto rosbag_dir = ros2::expandUser(tobas::kRosbagDirHome);
  if (!fs::is_directory(rosbag_dir)) {
    fs::create_directories(rosbag_dir);
  }

  try {
    for (const auto& entry : fs::directory_iterator(rosbag_dir)) {
      if (fs::remove_all(entry.path()) == 0) {
        qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(entry.path()));
        return;
      }
    }
  }
  catch (const std::exception& e) {
    qt::qErrorBox(
      this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + QString(e.what()));
    return;
  }

  if (log_list_->currentItem()) {
    log_list_->deselect();
    Q_EMIT logDeselected();
  }

  clearLogs();
}

void FlightLogsWidgetGCS::onExportFinished()
{
  spinner_.stop();
  qt::qInfoBox(this, "Flight log has been exported successfully.");
}

void FlightLogsWidgetGCS::onExportError(const QString& err)
{
  spinner_.stop();
  qt::qErrorBox(this, "Export Error" + err);
}

void FlightLogsWidgetGCS::onExportButtonClicked(const QString& log_name)
{
  fs::path defaultCsvName(log_name.toStdString());
  defaultCsvName.replace_extension(".csv");

  QString savePath = QFileDialog::getSaveFileName(
    this, "Select the directory", QString::fromStdString(defaultCsvName.string()), "CSV Files (*.csv)");

  if (savePath.isEmpty()) {
    return;
  }

  spinner_.start();
  QThread* thread = new QThread;
  CsvExportWorker* worker = new CsvExportWorker();

  connect(thread, &QThread::started, worker, [worker, log_name, savePath]() { worker->process(log_name, savePath); });

  // 処理完了時の後始末
  connect(worker, &CsvExportWorker::finished, thread, &QThread::quit);
  connect(worker, &CsvExportWorker::finished, worker, &CsvExportWorker::deleteLater);
  connect(thread, &QThread::finished, thread, &QThread::deleteLater);

  // GUIへの通知接続
  connect(worker, &CsvExportWorker::finished, this, &FlightLogsWidgetGCS::onExportFinished);
  connect(worker, &CsvExportWorker::error, this, &FlightLogsWidgetGCS::onExportError);

  // 4. スレッド開始
  thread->start();
}

void FlightLogsWidgetGCS::onDeleteButtonClicked(const QString& log_name)
{
  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  if (!qt::yesOrNo(this, "Do you want to delete flight log \"" + log_name + "\"?", qt::WARN)) {
    return;
  }

  if (fs::remove_all(log_path) == 0) {
    qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(log_path));
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
  const auto log_widget = qt::qConstPointerCast<FlightLogItemWidgetGCS>(log_list_->itemWidget(item));
  Q_EMIT logSelected(log_widget->logName());
}
}  // namespace log
}  // namespace gui
