#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_flight_log_gui/flight_log_reader.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogReaderWidget::FlightLogReaderWidget(rclcpp::Node::SharedPtr node)
  : read_thread_(node), download_thread_(node), clean_thread_(node), spinner_(Qt::WindowModal, this)
{
  read_button_ = new QPushButton("Read");
  download_button_ = new QPushButton("Download");
  clean_button_ = new QPushButton("Clean");

  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  download_button_->setFixedSize(kButtonWidth, kButtonHeight);
  clean_button_->setFixedSize(kButtonWidth, kButtonHeight);

  read_button_->setEnabled(true);
  download_button_->setEnabled(false);
  clean_button_->setEnabled(false);

  rosbag_list_ = new qt::ListWidget();
  rosbag_list_->setSelectionMode(QListWidget::SingleSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(read_button_);
  cols->addWidget(download_button_);
  cols->addWidget(clean_button_);
  cols->addStretch();

  const auto rows = new QVBoxLayout();
  rows->addLayout(cols);
  rows->addWidget(rosbag_list_);
  rows->addStretch();

  setLayout(rows);

  // Connections
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(download_button_, &QPushButton::clicked, this, &self::onDownloadButtonClicked);
  connect(clean_button_, &QPushButton::clicked, this, &self::onCleanButtonClicked);
  connect(&read_thread_, &ReadThread::finished, this, &self::onReadFinished);
  connect(&download_thread_, &DownloadThread::finished, this, &self::onDownloadFinished);
  connect(&clean_thread_, &CleanThread::finished, this, &self::onCleanFinished);
}

void FlightLogReaderWidget::onReadButtonClicked()
{
  read_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogReaderWidget::onDownloadButtonClicked()
{
  const auto item = rosbag_list_->selectedItem();
  if (item == nullptr)
  {
    qt::qWarnBox(this, "Please select the name of the log file that you want to download.");
    return;
  }

  const auto rosbag_name = item->text();
  const auto rosbag_path = ros2::expandUser(tobas::kROSBagDirHome) / rosbag_name.toStdString();

  if (fs::exists(rosbag_path))
  {
    if (qt::yesOrNo(
          this, QString(rosbag_path.c_str()) + " already exists. Do you want to overwrite it?",
          qt::QMessageLevel::WARN))
      fs::remove_all(rosbag_path);
    else
      return;
  }

  download_thread_.setROSBagName(rosbag_name);
  download_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogReaderWidget::onCleanButtonClicked()
{
  if (!qt::yesOrNo(this, "Do you want to clean all the flight logs saved in the FC?", qt::QMessageLevel::WARN))
    return;

  clean_thread_.start();

  spinner_.show();
  spinner_.start();
}

void FlightLogReaderWidget::onReadFinished(bool success, const QString& message, const QStringList& rosbag_names)
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

  download_button_->setEnabled(true);
  clean_button_->setEnabled(true);

  qt::qInfoBox(this, message);
}

void FlightLogReaderWidget::onDownloadFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (success)
    qt::qInfoBox(this, message);
  else
    qt::qErrorBox(this, message);
}

void FlightLogReaderWidget::onCleanFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success)
  {
    qt::qErrorBox(this, message);
    return;
  }

  rosbag_list_->clear();

  qt::qInfoBox(this, message);
}
}  // namespace log
}  // namespace gui
