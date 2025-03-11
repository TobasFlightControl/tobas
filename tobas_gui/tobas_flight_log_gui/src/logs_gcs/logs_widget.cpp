#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_flight_log_gui/logs_gcs/logs_widget.hpp"
#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"
#include "tobas_flight_log_gui/constants.hpp"

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
  cols->addWidget(read_button_);
  cols->addWidget(clean_button_);
  cols->addStretch();

  const auto rows = new QVBoxLayout();
  qt::addWidgetCenter(new qt::Label("Ground Control Station", kPSize1, QFont::Bold), rows);
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
  connect(widget, &FlightLogItemWidgetGCS::deleteButtonClicked, this, &self::onDeleteButtonClicked);
  log_list_->setItemWidget(list_item, widget);
}

void FlightLogsWidgetGCS::removeLog(const QString& log_name)
{
  const auto list_item = findLog(log_name);
  TOBAS_CHECK(list_item != nullptr);
  log_list_->remove(list_item);
}

QListWidgetItem* FlightLogsWidgetGCS::findLog(const QString& log_name)
{
  for (int row = 0; row < log_list_->count(); ++row)
  {
    const auto list_item = log_list_->item(row);
    const auto log_widget = qobject_cast<FlightLogItemWidgetGCS*>(log_list_->itemWidget(list_item));
    TOBAS_CHECK(log_widget != nullptr);

    if (log_widget->logName() == log_name)
      return list_item;
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

void FlightLogsWidgetGCS::onReadButtonClicked()
{
  clearLogs();

  const auto rosbag_dir = ros2::expandUser(tobas::kRosbagDirHome);
  if (!fs::is_directory(rosbag_dir))
    fs::create_directories(rosbag_dir);

  try
  {
    for (const auto& entry : fs::directory_iterator(rosbag_dir))
    {
      const QString log_name(entry.path().filename().c_str());
      addLog(log_name);
    }
  }
  catch (const std::exception& e)
  {
    qt::qErrorBox(
      this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + QString(e.what()));
    return;
  }

  if (log_list_->count() == 0)
  {
    qt::qWarnBox(this, "There are no flight logs saved on the ground control station.");
    return;
  }

  sortLogs();

  clean_button_->setEnabled(true);
}

void FlightLogsWidgetGCS::onCleanButtonClicked()
{
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the GCS?", qt::QMessageLevel::WARN))
    return;

  const auto rosbag_dir = ros2::expandUser(tobas::kRosbagDirHome);
  if (!fs::is_directory(rosbag_dir))
    fs::create_directories(rosbag_dir);

  try
  {
    for (const auto& entry : fs::directory_iterator(rosbag_dir))
    {
      if (!fs::remove_all(entry.path()))
      {
        qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(entry.path()));
        return;
      }
    }
  }
  catch (const std::exception& e)
  {
    qt::qErrorBox(
      this, "Exception occurred while iterating " + QString::fromStdString(rosbag_dir) + ": " + QString(e.what()));
    return;
  }

  clearLogs();
}

void FlightLogsWidgetGCS::onDeleteButtonClicked(const QString& log_name)
{
  const auto log_path = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  if (!qt::yesOrNo(this, "Do you want to delete flight log \"" + log_name + "\"?", qt::QMessageLevel::WARN))
    return;

  if (!fs::remove_all(log_path))
  {
    qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(log_path));
    return;
  }

  removeLog(log_name);
}

void FlightLogsWidgetGCS::onListItemClicked(QListWidgetItem* item)
{
  const auto log_widget = qobject_cast<FlightLogItemWidgetGCS*>(log_list_->itemWidget(item));
  Q_EMIT logSelected(log_widget->logName());
}
}  // namespace log
}  // namespace gui
