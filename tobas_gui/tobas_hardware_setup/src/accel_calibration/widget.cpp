#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/accel_calibration/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hardware_setup
{
AccelCalibrationWidget::AccelCalibrationWidget(rclcpp::Node::SharedPtr node)
  : spinner_(Qt::WindowModal, this), thread_(node)
{
}

const char* AccelCalibrationWidget::name() const
{
  return "Accel Calibration";
}

const char* AccelCalibrationWidget::title() const
{
  return "Calibrate Accelerometer";
}

void AccelCalibrationWidget::onInit()
{
  const auto instruction = new qt::DescriptionWidget(
    "Press \"Start\" button with the flight controller\'s TOP surface facing up.\n\n", kBodyPSize);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  // TODO: Rvizで重力方向と測定結果を表示

  // Layout
  rows_->addWidget(instruction);
  rows_->addWidget(start_button_);
  rows_->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&thread_, &AccelCalibrationThread::finished, this, &self::onCalibrationFinished);

  setEnabled(false);
}

void AccelCalibrationWidget::setNamespace(const std::string& ns)
{
  thread_.setNamespace(ns);
  setEnabled(true);
}

void AccelCalibrationWidget::onStartButtonClicked()
{
  spinner_.show();
  spinner_.start();

  thread_.start();
}

void AccelCalibrationWidget::onCalibrationFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (success)
    qt::qInfoBox(this, message);
  else
    qt::qErrorBox(this, message);
}
}  // namespace hardware_setup
}  // namespace gui
