#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>

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
    imu_raw_accel_x[m/s^2], imu_raw_accel_y[m/s^2], imu_raw_accel_z[m/s^2],\
    imu_raw_gyro_x[rad/s], imu_raw_gyro_y[rad/s], imu_raw_gyro_z[rad/s],\
    imu_raw_dgyro_x[rad/s^2], imu_raw_dgyro_y[rad/s^2], imu_raw_dgyro_z[rad/s^2]\n";

Q_SIGNALS:
  void logSelected(const QString& log_name);
  void logDeselected();

public:
  explicit FlightLogsWidgetGCS();

  void convertRosbag2CSV(const QString& log_name, const std::string& output_csv_path, const std::string& target_topic);
  void addLog(const QString& log_name);
  void removeLog(const QString& log_name);
  QListWidgetItem* findLog(const QString& log_name);

  void clearLogs();
  void sortLogs();

private:
  QPushButton* read_button_;
  QPushButton* clean_button_;

  qt::ListWidget* log_list_;

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
