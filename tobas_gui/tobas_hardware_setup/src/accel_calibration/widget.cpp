#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/accel_calibration/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hardware_setup
{
AccelCalibrationWidget::AccelCalibrationWidget(rclcpp::Node::SharedPtr node)
  : node_(node), spinner_(Qt::WindowModal, this), thread_(node)
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
  start_button_->setFixedSize(100, 40);

  // TODO: Rvizで重力方向と測定結果を表示

  // Layout
  rows_->addWidget(instruction);
  rows_->addWidget(start_button_);
  rows_->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&thread_, &AccelCalibrationThread::finished, this, &self::onCalibrationFinished);

  // ドローンが得られるまでは無効
  setEnabled(false);
}

void AccelCalibrationWidget::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
  setEnabled(true);
}

void AccelCalibrationWidget::onStartButtonClicked()
{
  if (drone_ == nullptr)
  {
    qt::qWarnBox(this, "Drone configuration is not received yet.");
    return;
  }

  spinner_.show();
  spinner_.start();

  thread_.setNamespace(drone_->name);
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
