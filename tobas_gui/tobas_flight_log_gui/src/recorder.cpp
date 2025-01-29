#include <filesystem>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_std_tools/string.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/srv/bag_record_start.hpp>
#include <tobas_msgs/srv/bag_record_stop.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_flight_log_gui/recorder.hpp"
#include "tobas_flight_log_gui/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogRecorderWidget::FlightLogRecorderWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
  log_name_ = new QLineEdit();

  start_button_ = new QPushButton("Start Recording");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  start_button_->setEnabled(false);

  stop_button_ = new QPushButton("Stop Recording");
  stop_button_->setFixedSize(kButtonWidth, kButtonHeight);
  stop_button_->setEnabled(false);

  duration_ = new QLCDNumber(8);
  duration_->setSegmentStyle(QLCDNumber::Flat);

  file_size_ = new qt::HPositionBarWidget();
  file_size_->setLower(0);
  file_size_->setMinimum(0);
  file_size_->setMaximum(tobas::kMaxRosbagSize);

  message_count_ = new qt::FramedLabel();
  message_count_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  // Layout
  const auto name_cols = new QHBoxLayout();
  name_cols->addWidget(new qt::Label("Log Name", kPSize2));
  name_cols->addWidget(log_name_);

  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(start_button_);
  button_cols->addWidget(stop_button_);
  button_cols->addStretch();
  button_cols->addWidget(duration_);

  const auto state_form = new qt::FormLayout();
  state_form->addVAlignedRow(new qt::Label("File Size", kPSize2), file_size_);
  state_form->addVAlignedRow(new qt::Label("Message Count", kPSize2), message_count_);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(name_cols);
  root_rows->addLayout(button_cols);
  root_rows->addLayout(state_form);
  root_rows->addStretch();

  setLayout(root_rows);

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(stop_button_, &QPushButton::clicked, this, &self::onStopButtonClicked);

  clearRosbagStateViewerWidgets();
  setEnabled(false);
}

void FlightLogRecorderWidget::updateNamespace(const std::string& ns)
{
  ns_ = ns;

  rosbag_state_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kRosbagStateTopic), &self::rosbagStateCb, this);

  clearRosbagStateViewerWidgets();
  setEnabled(true);
}

void FlightLogRecorderWidget::clearRosbagStateViewerWidgets()
{
  duration_->display("00:00:00");

  file_size_->setUpper(0);
  file_size_->setText("0 MB");

  message_count_->setText("0");
}

void FlightLogRecorderWidget::rosbagStateCb(const tobas_msgs::msg::RosbagState::ConstSharedPtr& rosbag_state)
{
  if (rosbag_state->recording)
  {
    log_name_->setText(fs::path(rosbag_state->file_path).lexically_normal().filename().c_str());
    log_name_->setEnabled(false);

    start_button_->setEnabled(false);
    stop_button_->setEnabled(true);

    const auto& total_secs = rosbag_state->duration.sec;
    const auto hours = total_secs / 3600;
    const auto minutes = (total_secs % 3600) / 60;
    const auto seconds = total_secs % 60;
    const auto hhmmss = QString("%1:%2:%3")
                          .arg(hours, 2, 10, QLatin1Char('0'))
                          .arg(minutes, 2, 10, QLatin1Char('0'))
                          .arg(seconds, 2, 10, QLatin1Char('0'));
    duration_->display(hhmmss);

    file_size_->setUpper(rosbag_state->file_size);
    file_size_->setText(QString::number(rosbag_state->file_size / 1'000'000) + " MB");

    message_count_->setText(QString::number(rosbag_state->message_count));
  }
  else
  {
    log_name_->setEnabled(true);
    start_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    clearRosbagStateViewerWidgets();
  }
}

void FlightLogRecorderWidget::onStartButtonClicked()
{
  const auto log_name = log_name_->text().toStdString();

  if (log_name.empty())
  {
    qt::qWarnBox(this, "Please specify the name of log file.");
    return;
  }

  if (!tobas_std::isValidFileName(log_name))
  {
    qt::qWarnBox(this, "The name of the log file is invalid.");
    return;
  }

  ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStart> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, tobas::kROSBagRecordStartSrv));

  const auto req = std::make_shared<tobas_msgs::srv::BagRecordStart::Request>();
  req->name = log_name;

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

  log_name_->setEnabled(false);
  start_button_->setEnabled(false);
  clearRosbagStateViewerWidgets();

  qt::qInfoBox(this, "Flight log recording has started.");
}

void FlightLogRecorderWidget::onStopButtonClicked()
{
  ros2::SyncServiceClient<tobas_msgs::srv::BagRecordStop> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, tobas::kROSBagRecordStopSrv));

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

  log_name_->clear();
  stop_button_->setEnabled(false);
  clearRosbagStateViewerWidgets();

  qt::qInfoBox(this, "Flight log recording has stopped.");
}
}  // namespace log
}  // namespace gui
