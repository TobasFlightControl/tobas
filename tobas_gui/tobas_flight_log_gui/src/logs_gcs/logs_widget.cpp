#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_flight_log_gui/logs_gcs/logs_widget.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogsWidgetGCS::FlightLogsWidgetGCS()
{
  read_button_ = new QPushButton("Read");
  delete_button_ = new QPushButton("Delete");
  clean_button_ = new QPushButton("Clean");

  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  delete_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  read_button_->setEnabled(true);
  delete_button_->setEnabled(false);
  clean_button_->setEnabled(false);

  rosbag_list_ = new qt::ListWidget();
  rosbag_list_->setSelectionMode(QListWidget::SingleSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(read_button_);
  cols->addWidget(delete_button_);
  cols->addWidget(clean_button_);
  cols->addStretch();

  const auto rows = new QVBoxLayout();
  qt::addWidgetCenter(new qt::Label("Ground Control Station", 18, QFont::Bold), rows);
  rows->addLayout(cols);
  rows->addWidget(rosbag_list_);

  setLayout(rows);

  // Connections
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(delete_button_, &QPushButton::clicked, this, &self::onDeleteButtonClicked);
  connect(clean_button_, &QPushButton::clicked, this, &self::onCleanButtonClicked);
}

void FlightLogsWidgetGCS::read()
{
  rosbag_list_->clear();

  for (const auto& entry : fs::directory_iterator(ros2::expandUser(tobas::kROSBagDirHome)))
    rosbag_list_->addItem(QString::fromStdString(entry.path().filename().string()));

  rosbag_list_->sortItems();
}

void FlightLogsWidgetGCS::onReadButtonClicked()
{
  read();

  if (rosbag_list_->count() == 0)
  {
    qt::qWarnBox(this, "There are no flight logs saved on the ground control station.");
    return;
  }

  delete_button_->setEnabled(true);
  clean_button_->setEnabled(true);
}

void FlightLogsWidgetGCS::onDeleteButtonClicked()
{
  const auto item = rosbag_list_->selectedItem();
  if (item == nullptr)
  {
    qt::qWarnBox(this, "Please select the name of the log file that you want to delete.");
    return;
  }

  const auto rosbag_name = item->text();
  const auto rosbag_path = ros2::expandUser(tobas::kROSBagDirHome) / rosbag_name.toStdString();

  if (!qt::yesOrNo(this, "Do you want to delete " + rosbag_name + "?", qt::QMessageLevel::WARN))
    return;

  if (!fs::remove_all(rosbag_path))
  {
    qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(rosbag_path));
    return;
  }

  rosbag_list_->remove(item);
}

void FlightLogsWidgetGCS::onCleanButtonClicked()
{
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the GCS?", qt::QMessageLevel::WARN))
    return;

  for (const auto& entry : fs::directory_iterator(ros2::expandUser(tobas::kROSBagDirHome)))
  {
    if (!fs::remove_all(entry.path()))
    {
      qt::qErrorBox(this, "Failed to delete " + QString::fromStdString(entry.path()));
      read();
      return;
    }
  }

  rosbag_list_->clear();
}
}  // namespace log
}  // namespace gui
