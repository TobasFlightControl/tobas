#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/rcin_calibration/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

using namespace std;

namespace gui
{
namespace hardware_setup
{
RCInputCalibrationWidget::RCInputCalibrationWidget(rclcpp::Node::SharedPtr node) : node_(node)
{
}

const char* RCInputCalibrationWidget::name() const
{
  return "Radio Calibration";
}

const char* RCInputCalibrationWidget::title() const
{
  return "Calibrate RC Input";
}

void RCInputCalibrationWidget::onInit()
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Press \"Start\" button.\n\n"
    "2. For each channel, operate the stick or switch to ensure it covers the entire range. "
    "If the stick's movement is opposite to that of the bar, adjust the transmitter settings accordingly.\n\n"
    "3. Press \"Finish\" button.\n\n",
    kBodyPSize);
  rows_->addWidget(instruction);

  const auto cols1 = new QHBoxLayout();
  rows_->addLayout(cols1);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  cols1->addWidget(start_button_);

  finish_button_ = new QPushButton("Finish");
  finish_button_->setFixedSize(kButtonWidth, kButtonHeight);
  finish_button_->setEnabled(false);
  connect(finish_button_, &QPushButton::clicked, this, &self::onFinishButtonClicked);
  cols1->addWidget(finish_button_);

  cancel_button_ = new QPushButton("Cancel");
  cancel_button_->setFixedSize(kButtonWidth, kButtonHeight);
  cancel_button_->setEnabled(false);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  cols1->addWidget(cancel_button_);

  cols1->addStretch();

  rows_->addSpacing(50);

  const auto cols2 = new QHBoxLayout();
  rows_->addLayout(cols2);

  // Roll, Pitch, Yaw, Throttle
  const auto cols3 = qt::createFixedHeightQHBoxLayout(kRangeSideLong + 20, cols2);

  pitch_range_ = new qt::VPositionBarWidget(kMinThrot, kMaxThrot);
  pitch_range_->setFixedSize(kRangeSideShort, kRangeSideLong);
  cols3->addWidget(pitch_range_);

  const auto rows1 = new QVBoxLayout();
  cols3->addLayout(rows1);

  roll_range_ = new qt::HPositionBarWidget(kMinThrot, kMaxThrot);
  roll_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  qt::addWidgetCenter(roll_range_, rows1);
  qt::addWidgetCenter(new QLabel(QString::fromStdString(format("Roll (CH{})", real::kRcChannelRoll + 1))), rows1);

  rows1->addStretch();

  const auto cols4 = new QHBoxLayout();
  rows1->addLayout(cols4);

  const auto pitch_label = new QLabel(QString::fromStdString(format("Pitch (CH{})", real::kRcChannelPitch + 1)));
  pitch_label->setAlignment(Qt::AlignLeft);
  cols4->addWidget(pitch_label);

  const auto throttle_label = new QLabel(QString::fromStdString(format("Throttle (CH{})", real::kRcChannelThrot + 1)));
  throttle_label->setAlignment(Qt::AlignRight);
  cols4->addWidget(throttle_label);

  rows1->addStretch();

  qt::addWidgetCenter(new QLabel(QString::fromStdString(format("Yaw (CH{})", real::kRcChannelYaw + 1))), rows1);
  yaw_range_ = new qt::HPositionBarWidget(kMinThrot, kMaxThrot);
  yaw_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  qt::addWidgetCenter(yaw_range_, rows1);

  throt_range_ = new qt::VPositionBarWidget(kMinThrot, kMaxThrot);
  throt_range_->setFixedSize(kRangeSideShort, kRangeSideLong);
  cols3->addWidget(throt_range_);

  cols2->addSpacing(30);

  const auto bar_grid = new QGridLayout();
  cols2->addLayout(bar_grid);

  // Mode
  bar_grid->addWidget(new QLabel(QString::fromStdString(format("Mode (CH{})", real::kRcChannelMode + 1))), 0, 0);
  mode_range_ = new qt::HPositionBarWidget(kMinThrot, kMaxThrot);
  mode_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  bar_grid->addWidget(mode_range_, 0, 1);

  // E-Stop
  bar_grid->addWidget(new QLabel(QString::fromStdString(format("E-Stop (CH{})", real::kRcChannelEStop + 1))), 1, 0);
  estop_range_ = new qt::HPositionBarWidget(kMinThrot, kMaxThrot);
  estop_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  bar_grid->addWidget(estop_range_, 1, 1);

  // GPSw
  bar_grid->addWidget(new QLabel(QString::fromStdString(format("GPSw (CH{})", real::kRcChannelGPSw + 1))), 2, 0);
  gpsw_range_ = new qt::HPositionBarWidget(kMinThrot, kMaxThrot);
  gpsw_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  bar_grid->addWidget(gpsw_range_, 2, 1);

  cols2->addStretch();
  rows_->addStretch();

  reset();

  setEnabled(false);
}

void RCInputCalibrationWidget::setNamespace(const string& ns)
{
  ns_ = ns;
  reset();
  setEnabled(true);
}

void RCInputCalibrationWidget::reset()
{
  if (sbus_sub_ != nullptr)
    sbus_sub_ = nullptr;

  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();
  mode_range_->clear();
  estop_range_->clear();
  gpsw_range_->clear();

  mode_range_->setText(kModeText);
  estop_range_->setText(kOnOffText);
  gpsw_range_->setText(kOnOffText);

  start_button_->setEnabled(true);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
}

void RCInputCalibrationWidget::sbusCb(const tobas_hal_msgs::msg::Sbus::ConstSharedPtr& sbus)
{
  roll_range_->setValue(sbus->data[real::kRcChannelRoll]);
  pitch_range_->setValue(sbus->data[real::kRcChannelPitch]);
  yaw_range_->setValue(sbus->data[real::kRcChannelYaw]);
  throt_range_->setValue(sbus->data[real::kRcChannelThrot]);
  mode_range_->setValue(sbus->data[real::kRcChannelMode]);
  estop_range_->setValue(sbus->data[real::kRcChannelEStop]);
  gpsw_range_->setValue(sbus->data[real::kRcChannelGPSw]);
}

void RCInputCalibrationWidget::onStartButtonClicked()
{
  // 一時的にSBUSトピックを購読開始
  sbus_sub_ = ros2::createSubscriber(node_, ns_ + "/" + hal::kSbusTopic, &self::sbusCb, this);

  start_button_->setEnabled(false);
  finish_button_->setEnabled(true);
  cancel_button_->setEnabled(true);

  qt::qInfoBox(this, "Radio calibration is started.");
}

void RCInputCalibrationWidget::onCancelButtonClicked()
{
  qt::qInfoBox(this, "Radio calibration is cancelled.");
  reset();
}

void RCInputCalibrationWidget::onFinishButtonClicked()
{
  // 各チャンネルの値の範囲をチェック
  if (roll_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Roll channel is too narrow.");
    reset();
    return;
  }
  if (pitch_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Pitch channel is too narrow.");
    reset();
    return;
  }
  if (yaw_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Yaw channel is too narrow.");
    reset();
    return;
  }
  if (throt_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Throttle channel is too narrow.");
    reset();
    return;
  }
  if (mode_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Mode channel is too narrow.");
    reset();
    return;
  }
  if (estop_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of E-Stop channel is too narrow.");
    reset();
    return;
  }
  if (gpsw_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of GPSw channel is too narrow.");
    reset();
    return;
  }

  // パラメータを作成
  vector<double> params(real::handler::rcin::kParamSize);
  params.at(real::handler::rcin::kRollLeftChannel) = roll_range_->getLower();
  params.at(real::handler::rcin::kRollRightChannel) = roll_range_->getUpper();
  params.at(real::handler::rcin::kPitchUpChannel) = pitch_range_->getLower();
  params.at(real::handler::rcin::kPitchDownChannel) = pitch_range_->getUpper();
  params.at(real::handler::rcin::kYawLeftChannel) = yaw_range_->getLower();
  params.at(real::handler::rcin::kYawRightChannel) = yaw_range_->getUpper();
  params.at(real::handler::rcin::kThrotUpChannel) = throt_range_->getLower();
  params.at(real::handler::rcin::kThrotDownChannel) = throt_range_->getUpper();
  params.at(real::handler::rcin::kModeProgramChannel) = mode_range_->getLower();
  params.at(real::handler::rcin::kModeStabilizeChannel) = mode_range_->getMiddle();
  params.at(real::handler::rcin::kModeAcrobatChannel) = mode_range_->getUpper();
  params.at(real::handler::rcin::kEStopOnChannel) = estop_range_->getLower();
  params.at(real::handler::rcin::kEStopOffChannel) = estop_range_->getUpper();
  params.at(real::handler::rcin::kGPSwOnChannel) = gpsw_range_->getLower();
  params.at(real::handler::rcin::kGPSwOffChannel) = gpsw_range_->getUpper();

  // パラメータを更新
  ros2::SyncParamClient param_client(node_, ns_ + "/rcin_handler");
  if (!param_client.setParam(real::handler::kParamName, params, kSetParamTimeout))
  {
    qt::qErrorBox(this, "Failed to send calibration results.");
    reset();
    return;
  }

  qt::qInfoBox(this, "Radio calibration finished successfully.");
  reset();
}
}  // namespace hardware_setup
}  // namespace gui
