#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/adc_calibration/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hardware_setup
{
ADCCalibrationWidget::ADCCalibrationWidget(rclcpp::Node::SharedPtr node)
  : node_(node), spinner_(Qt::WindowModal, this), thread_(node)
{
}

const char* ADCCalibrationWidget::name() const
{
  return "ADC Calibration";
}

const char* ADCCalibrationWidget::title() const
{
  return "Calibrate ADCerometer";
}

void ADCCalibrationWidget::onInit()
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Ensure the battery and the ADC port is correctly connected.\n\n"
    "2. Input the current battery voltage.\n\n"
    "3. Press \"Start\" button.\n\n",
    kBodyPSize);
  rows_->addWidget(instruction);

  const auto cols1 = new QHBoxLayout();
  rows_->addLayout(cols1);

  cols1->addWidget(new QLabel("Voltage:"));

  voltage_ = new qt::DoubleSpinBox();
  voltage_->setSuffix(" V");
  voltage_->setFixedWidth(kBoxWidth);
  voltage_->setMinimum(0.0);
  cols1->addWidget(voltage_);

  cols1->addStretch();

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  rows_->addWidget(start_button_);

  rows_->addSpacing(50);

  const auto cols2 = new QHBoxLayout();
  rows_->addLayout(cols2);

  cols2->addWidget(new QLabel("ADC Coefficient:"));

  adc_coef_ = new QLineEdit();
  adc_coef_->setFixedWidth(kBoxWidth);
  adc_coef_->setReadOnly(true);
  adc_coef_->setFocusPolicy(Qt::NoFocus);
  cols2->addWidget(adc_coef_);

  cols2->addStretch();

  rows_->addStretch();

  // ドローンが得られるまでは無効
  setEnabled(false);

  drone_sub_ = ros2::createSubscriber(node_, tobas::kDroneTopic, &self::droneCb, this, true, true);
}

void ADCCalibrationWidget::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
  setEnabled(true);
}

void ADCCalibrationWidget::onStartButtonClicked()
{
  if (drone_ == nullptr)
  {
    qt::qWarnBox(this, "Drone configuration is not received yet.");
    return;
  }

  if (voltage_->value() <= 0.)
  {
    qt::qWarnBox(this, "Please set positive battery voltage.");
  }

  spinner_.show();
  spinner_.start();

  thread_.setNamespace(drone_->name);
  thread_.setCurrentVoltage(voltage_->value());

  thread_.start();
}

void ADCCalibrationWidget::onCalibrationFinished(bool success, const QString& message)
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
