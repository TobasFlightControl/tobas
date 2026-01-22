#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include <rosbag2_cpp/reader.hpp>

#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/odometry.hpp>

namespace gui
{
namespace log
{
class FlightLogsWidgetGCS : public QWidget
{
  Q_OBJECT

  using self = FlightLogsWidgetGCS;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kListItemHeight = 40;

  std::string exportCSVHeader = "time,\
    imu_raw/accel/x, imu_raw/accel/y imu_raw/accel/z,\
    imu_raw/gyro/x, imu_raw/gyro/y, imu_raw/gyro/z,\
    imu_raw/dgyro/x, imu_raw/dgyro/y, imu_raw/dgyro/z,\
    battery/voltage, battery/current\n";

Q_SIGNALS:
  void logSelected(const QString& log_name);
  void logDeselected();

public:
  explicit FlightLogsWidgetGCS();

  struct curData{
    tobas_msgs::msg::Odometry cur_odom_data;
    tobas_msgs::msg::Imu cur_Imu_data;
    tobas_msgs::msg::Battery cur_battery;
  }curData_;

  std::string makeCSVRow(const auto& cur_time);
  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  bool open(const std::string& rosbag_path);
  bool reindex(const std::string& rosbag_path);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();
  void sortLogs();

private:
  rosbag2_cpp::Reader reader_;
  
  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

  qt::WaitSpinnerWidget spinner_;

  QString currentLogName() const;
  void setCurrentLogName(const QString& log_name);

private Q_SLOTS:
  void onReadButtonClicked();
  void onCleanButtonClicked();
  void onExportButtonClicked(const QString& log_name);
  void onDeleteButtonClicked(const QString& log_name);
  void onListItemClicked(QListWidgetItem* item);
};
}  // namespace log
}  // namespace gui
