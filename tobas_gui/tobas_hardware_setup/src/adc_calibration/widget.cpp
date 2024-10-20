#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>
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
  const auto instruction = new qt::DescriptionWidget(
    "1. Ensure the battery and the ADC port is correctly connected.\n\n"
    "2. Input the current battery voltage.\n\n"
    "3. Press \"Start\" button.\n\n",
    kBodyPSize);

  voltage_ = new qt::DoubleSpinBox();
  voltage_->setSuffix(" V");
  voltage_->setFixedWidth(kBoxWidth);
  voltage_->setMinimum(0.0);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  adc_coef_ = new QLineEdit();
  adc_coef_->setFixedWidth(kBoxWidth);
  adc_coef_->setReadOnly(true);
  adc_coef_->setFocusPolicy(Qt::NoFocus);

  // Layout
  const auto cols1 = new QHBoxLayout();
  cols1->addWidget(new QLabel("Voltage:"));
  cols1->addWidget(voltage_);
  cols1->addStretch();

  const auto cols2 = new QHBoxLayout();
  cols2->addWidget(new QLabel("ADC Coefficient:"));
  cols2->addWidget(adc_coef_);
  cols2->addStretch();

  rows_->addWidget(instruction);
  rows_->addLayout(cols1);
  rows_->addWidget(start_button_);
  rows_->addSpacing(50);
  rows_->addLayout(cols2);
  rows_->addStretch();

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(&thread_, &ADCCalibrationThread::finished, this, &self::onCalibrationFinished);

  setEnabled(false);
}

const char* ADCCalibrationWidget::name() const
{
  return "ADC Calibration";
}

const char* ADCCalibrationWidget::title() const
{
  return "Calibrate Battery A/D Converter";
}

void ADCCalibrationWidget::setNamespace(const std::string& ns)
{
  thread_.setNamespace(ns);

  arming_ = nullptr;
  arming_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);

  setEnabled(true);
}

void ADCCalibrationWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void ADCCalibrationWidget::onStartButtonClicked()
{
  // アームされていないことを確認
  if (arming_ == nullptr)
  {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status is not received yet.");
    return;
  }
  if (arming_->data)
  {
    qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
    return;
  }

  if (voltage_->value() <= 0.)
  {
    qt::qWarnBox(this, "Please set positive battery voltage.");
    return;
  }

  spinner_.show();
  spinner_.start();

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
