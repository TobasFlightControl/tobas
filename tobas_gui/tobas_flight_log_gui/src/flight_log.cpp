#include <QVBoxLayout>
#include <QHBoxLayout>

#include "tobas_flight_log_gui/flight_log.hpp"

namespace gui
{
namespace log
{
FlightLogWidget::FlightLogWidget(rclcpp::Node::SharedPtr node) : node_(node), ssh_client_(node)
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
  // TODO
}

void FlightLogWidget::onReadButtonClicked()
{
  // TODO
}

void FlightLogWidget::onDownloadButtonClicked()
{
  // TODO
}

void FlightLogWidget::onCleanButtonClicked()
{
  // TODO
}

void FlightLogWidget::onReadFinished()
{
  // TODO
}

void FlightLogWidget::onDownloadFinished()
{
  // TODO
}

void FlightLogWidget::onCleanFinished()
{
  // TODO
}
}  // namespace log
}  // namespace gui
