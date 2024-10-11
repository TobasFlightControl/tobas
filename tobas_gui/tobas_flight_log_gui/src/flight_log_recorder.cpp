#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_std_tools/string.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_flight_log_gui/flight_log_recorder.hpp"

namespace gui
{
namespace log
{
FlightLogRecorderWidget::FlightLogRecorderWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  rosbag_name_ = new QLineEdit();

  start_button_ = new QPushButton("Start Recording");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  stop_button_ = new QPushButton("Stop Recording");
  stop_button_->setFixedSize(kButtonWidth, kButtonHeight);
  stop_button_->setEnabled(false);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(start_button_);
  cols->addWidget(stop_button_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(rosbag_name_);
  rows->addLayout(cols);

  setLayout(rows);
}

void FlightLogRecorderWidget::updateNamespace(const std::string& ns)
{
  ns_ = ns;
}

void FlightLogRecorderWidget::onStartButtonClicked()
{
  const auto rosbag_name = rosbag_name_->text().toStdString();

  if (rosbag_name.empty())
  {
    qt::qWarnBox(this, "Please specify the name of log file.");
    return;
  }

  if (!tobas_std::isValidFileName(rosbag_name))
  {
    qt::qWarnBox(this, "The name of the log file is invalid.");
    return;
  }

  ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStart> sc(node_, path::join(ns_, tobas::kROSBagRecordStartSrv));

  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStart::Request>();
  req->name = rosbag_name;

  if (!sc.call(req))
  {
    qt::qErrorBox(this, "Flight log recording service is unavailable.");
    return;
  }

  const auto res = sc.getResponse();
  if (!res->success)
  {
    qt::qErrorBox(this, "Failed to start recording flight log: " + QString(res->message.c_str()));
    return;
  }

  rosbag_name_->setEnabled(false);
  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);

  qt::qInfoBox(this, "Flight log recording has started.");
}

void FlightLogRecorderWidget::onStopButtonClicked()
{
  ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStop> sc(node_, path::join(ns_, tobas::kROSBagRecordStopSrv));

  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStop::Request>();

  if (!sc.call(req))
  {
    qt::qErrorBox(this, "Flight log recording service is unavailable.");
    return;
  }

  const auto res = sc.getResponse();
  if (!res->success)
  {
    qt::qErrorBox(this, "Failed to stop recording flight log: " + QString(res->message.c_str()));
    return;
  }

  rosbag_name_->setEnabled(true);
  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);

  qt::qInfoBox(this, "Flight log recording has stopped.");
}
}  // namespace log
}  // namespace gui
