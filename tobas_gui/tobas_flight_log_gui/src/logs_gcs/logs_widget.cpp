#include "tobas_flight_log_gui/logs_gcs/logs_widget.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_flight_log_gui/constants.hpp"
#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"

#include <QFileDialog>
#include <fstream>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include "tobas_kdl_msgs/msg/vector.hpp"
#include "tobas_msgs/msg/imu.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogsWidgetGCS::FlightLogsWidgetGCS()
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

void FlightLogsWidgetGCS::convertRosbag2CSV(
  const QString& log_name,
  const std::string& output_csv_path,
  const std::string& target_topic)
{
  std::ofstream csvFile(output_csv_path);
  if (!csvFile.is_open()) {
    std::cerr << "Couldn't write to csv file: " << output_csv_path << std::endl;
    return;
  }
  csvFile << FlightLogsWidgetGCS::exportCSVHeader;

  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();
  rosbag2_cpp::Reader reader;

  try {
    reader.open(log_path.string());
  }
  catch (const std::exception& e) {
    std::cerr << "Couldn't open rosbag: " << e.what() << std::endl;
    return;
  }

  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(target_topic);
  reader.set_filter(filter);

  rclcpp::Serialization<tobas_msgs::msg::Imu> serializer;
  tobas_msgs::msg::Imu raw_msg;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();

    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
    serializer.deserialize_message(&serialized_msg, &raw_msg);

    double timestamp = raw_msg.header.stamp.sec + (raw_msg.header.stamp.nanosec * 1e-9);

    csvFile << std::fixed << std::setprecision(9) << timestamp << "," << raw_msg.accel.x << "," << raw_msg.accel.y
            << "," << raw_msg.accel.z << "," << raw_msg.gyro.x << "," << raw_msg.gyro.y << "," << raw_msg.gyro.z << ","
            << raw_msg.dgyro.x << "," << raw_msg.dgyro.y << "," << raw_msg.dgyro.z << "\n";
  }
  csvFile.close();
}

void FlightLogsWidgetGCS::onExportButtonClicked(const QString& log_name)
{
  if (!qt::yesOrNo(this, "Do you want to export flight log \"" + log_name + " to csv\"?", qt::WARN)) {
    return;
  }

  fs::path defaultCsvName(log_name.toStdString());
  defaultCsvName.replace_extension(".csv");

  QString savePath = QFileDialog::getSaveFileName(
    this, "Select the directory", QString::fromStdString(defaultCsvName.string()), "CSV Files (*.csv)");

  if (savePath.isEmpty()) {
    return;
  }

  std::ofstream csvFile(savePath.toStdString());

  if (!csvFile.is_open()) {
    std::cerr << "Failed to save CSV file" << std::endl;
    return;
  }

  std::string imu_topic = "/f450/imu_raw";
  convertRosbag2CSV(log_name, savePath.toStdString(), imu_topic);
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
