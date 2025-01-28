#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_flight_log_gui/logs_fc/logs_widget.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogsWidgetFC::FlightLogsWidgetFC(rclcpp::Node::SharedPtr node)
  : load_thread_(node), delete_thread_(node), clean_thread_(node), spinner_(Qt::WindowModal, this)
{
  load_button_ = new QPushButton("Load");
  delete_button_ = new QPushButton("Delete");
  clean_button_ = new QPushButton("Clean");

  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  delete_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  load_button_->setEnabled(true);
  delete_button_->setEnabled(false);
  clean_button_->setEnabled(false);

  rosbag_list_ = new qt::ListWidget();
  rosbag_list_->setSelectionMode(QListWidget::SingleSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(load_button_);
  cols->addWidget(delete_button_);
  cols->addWidget(clean_button_);
  cols->addStretch();

  const auto rows = new QVBoxLayout();
  qt::addWidgetCenter(new qt::Label("Flight Controller", 18, QFont::Bold), rows);
  rows->addLayout(cols);
  rows->addWidget(rosbag_list_);

  setLayout(rows);

  // Connections
  connect(load_button_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(delete_button_, &QPushButton::clicked, this, &self::onDeleteButtonClicked);
  connect(clean_button_, &QPushButton::clicked, this, &self::onCleanButtonClicked);
  connect(&load_thread_, &LoadThreadFC::finished, this, &self::onLoadFinished);
  connect(&delete_thread_, &DeleteThreadFC::finished, this, &self::onDeleteFinished);
  connect(&clean_thread_, &CleanThreadFC::finished, this, &self::onCleanFinished);
}

void FlightLogsWidgetFC::onLoadButtonClicked()
{
  load_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogsWidgetFC::onDeleteButtonClicked()
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

  delete_thread_.setROSBagName(rosbag_name);
  delete_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogsWidgetFC::onCleanButtonClicked()
{
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the FC?", qt::QMessageLevel::WARN))
    return;

  clean_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogsWidgetFC::onLoadFinished(bool success, const QString& message, const QStringList& rosbag_names)
{
  spinner_.hide();
  spinner_.stop();

  if (!success)
  {
    qt::qErrorBox(this, message);
    return;
  }

  rosbag_list_->clear();

  if (rosbag_names.size() == 0)
  {
    qt::qWarnBox(this, "There are no flight logs saved on the flight controller.");
    return;
  }

  for (const auto& name : rosbag_names)
    rosbag_list_->addItem(name);

  delete_button_->setEnabled(true);
  clean_button_->setEnabled(true);
}

void FlightLogsWidgetFC::onDeleteFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success)
  {
    qt::qErrorBox(this, message);
    return;
  }

  rosbag_list_->remove(rosbag_list_->selectedItem());
}

void FlightLogsWidgetFC::onCleanFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success)
  {
    qt::qErrorBox(this, message);
    return;
  }

  rosbag_list_->clear();
}
}  // namespace log
}  // namespace gui
