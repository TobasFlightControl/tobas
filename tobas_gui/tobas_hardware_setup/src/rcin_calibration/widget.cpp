#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/rcin_calibration/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

using namespace std;
using namespace real::handler::rcin;

namespace gui
{
namespace hw
{
RCInputCalibrationWidget::RCInputCalibrationWidget(rclcpp::Node::SharedPtr node) : node_(node), rate_(kTopicRate)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Press \"Start\" button.\n\n"
    "2. For each channel, operate the stick or switch to ensure it covers the entire range. "
    "If the stick's movement is opposite to that of the bar, adjust the transmitter settings accordingly.\n\n"
    "3. Press \"Finish\" button.\n\n",
    kBodyPSize);
  rows_->addWidget(instruction);

  const auto button_cols = new QHBoxLayout();
  rows_->addLayout(button_cols);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  button_cols->addWidget(start_button_);

  finish_button_ = new QPushButton("Finish");
  finish_button_->setFixedSize(kButtonWidth, kButtonHeight);
  finish_button_->setEnabled(false);
  connect(finish_button_, &QPushButton::clicked, this, &self::onFinishButtonClicked);
  button_cols->addWidget(finish_button_);

  cancel_button_ = new QPushButton("Cancel");
  cancel_button_->setFixedSize(kButtonWidth, kButtonHeight);
  cancel_button_->setEnabled(false);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  button_cols->addWidget(cancel_button_);

  button_cols->addStretch();

  rows_->addSpacing(50);

  const auto rc_range_cols = new QHBoxLayout();
  rows_->addLayout(rc_range_cols);

  // Sticks
  const auto stick_cols = qt::createFixedHeightQHBoxLayout(kRangeSideLong + 20, rc_range_cols);

  pitch_range_ = new qt::VPositionBarWidget(kMinPeriod, kMaxPeriod);
  pitch_range_->setFixedSize(kRangeSideShort, kRangeSideLong);
  stick_cols->addWidget(pitch_range_);

  const auto roll_yaw_rows = new QVBoxLayout();
  stick_cols->addLayout(roll_yaw_rows);

  roll_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  roll_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  qt::addWidgetCenter(roll_range_, roll_yaw_rows);
  qt::addWidgetCenter(
    new QLabel(QString::fromStdString(format("Roll (CH{})", real::kRcChannelRoll + 1))), roll_yaw_rows);

  roll_yaw_rows->addStretch();

  const auto pitch_throt_label_cols = new QHBoxLayout();
  roll_yaw_rows->addLayout(pitch_throt_label_cols);

  const auto pitch_label = new QLabel(QString::fromStdString(format("Pitch (CH{})", real::kRcChannelPitch + 1)));
  pitch_label->setAlignment(Qt::AlignLeft);
  pitch_throt_label_cols->addWidget(pitch_label);

  const auto throt_label = new QLabel(QString::fromStdString(format("Throttle (CH{})", real::kRcChannelThrot + 1)));
  throt_label->setAlignment(Qt::AlignRight);
  pitch_throt_label_cols->addWidget(throt_label);

  roll_yaw_rows->addStretch();

  qt::addWidgetCenter(new QLabel(QString::fromStdString(format("Yaw (CH{})", real::kRcChannelYaw + 1))), roll_yaw_rows);
  yaw_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  yaw_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  qt::addWidgetCenter(yaw_range_, roll_yaw_rows);

  throt_range_ = new qt::VPositionBarWidget(kMinPeriod, kMaxPeriod);
  throt_range_->setFixedSize(kRangeSideShort, kRangeSideLong);
  stick_cols->addWidget(throt_range_);

  rc_range_cols->addSpacing(30);

  // Toggle Switches
  const auto switch_grid = new QGridLayout();
  rc_range_cols->addLayout(switch_grid);

  switch_grid->addWidget(new QLabel(QString::fromStdString(format("Enable (CH{})", real::kRcChannelEnable + 1))), 0, 0);
  enable_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  enable_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  switch_grid->addWidget(enable_range_, 0, 1);

  switch_grid->addWidget(new QLabel(QString::fromStdString(format("Kill (CH{})", real::kRcChannelKill + 1))), 1, 0);
  kill_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  kill_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  switch_grid->addWidget(kill_range_, 1, 1);

  switch_grid->addWidget(new QLabel(QString::fromStdString(format("Mode (CH{})", real::kRcChannelMode + 1))), 2, 0);
  mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  mode_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  switch_grid->addWidget(mode_range_, 2, 1);

  switch_grid->addWidget(
    new QLabel(QString::fromStdString(format("Sub Mode (CH{})", real::kRcChannelSubMode + 1))), 3, 0);
  sub_mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  sub_mode_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  switch_grid->addWidget(sub_mode_range_, 3, 1);

  switch_grid->addWidget(new QLabel(QString::fromStdString(format("GPSw 1 (CH{})", real::kRcChannelGPSw1 + 1))), 4, 0);
  gpsw1_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  gpsw1_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  switch_grid->addWidget(gpsw1_range_, 4, 1);

  switch_grid->addWidget(new QLabel(QString::fromStdString(format("GPSw 2 (CH{})", real::kRcChannelGPSw2 + 1))), 5, 0);
  gpsw2_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  gpsw2_range_->setFixedSize(kRangeSideLong, kRangeSideShort);
  switch_grid->addWidget(gpsw2_range_, 5, 1);

  rc_range_cols->addStretch();
  rows_->addStretch();

  reset();

  setEnabled(false);
}

const char* RCInputCalibrationWidget::name() const
{
  return "Radio Calibration";
}

const char* RCInputCalibrationWidget::title() const
{
  return "Calibrate RC Input";
}

void RCInputCalibrationWidget::reset()
{
  if (sbus_sub_ != nullptr)
    sbus_sub_ = nullptr;

  rate_.reset();

  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();
  enable_range_->clear();
  kill_range_->clear();
  mode_range_->clear();
  sub_mode_range_->clear();
  gpsw1_range_->clear();
  gpsw2_range_->clear();

  enable_range_->setText(kOnOffText);
  kill_range_->setText(kOnOffText);
  mode_range_->setText(kModeText);
  sub_mode_range_->setText(kOnOffText);
  gpsw1_range_->setText(kOnOffText);
  gpsw2_range_->setText(kOnOffText);

  start_button_->setEnabled(true);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
}

void RCInputCalibrationWidget::setNamespace(const string& ns)
{
  ns_ = ns;

  reset();

  arming_ = nullptr;
  arming_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);

  setEnabled(true);
}

bool RCInputCalibrationWidget::saveParamsGCS()
{
  ptree::PropertyTree pt;
  if (!pt.initialize((ros2::expandUser(tobas::kConfigDirHome) / kConfigFileName)))
  {
    qt::qErrorBox(this, "Failed to initialize property tree.");
    return false;
  }

  pt.set(kRollLeftKey, roll_range_->getLower());
  pt.set(kRollRightKey, roll_range_->getUpper());
  pt.set(kPitchUpKey, pitch_range_->getLower());
  pt.set(kPitchDownKey, pitch_range_->getUpper());
  pt.set(kYawLeftKey, yaw_range_->getLower());
  pt.set(kYawRightKey, yaw_range_->getUpper());
  pt.set(kThrotUpKey, throt_range_->getLower());
  pt.set(kThrotDownKey, throt_range_->getUpper());
  pt.set(kEnableOnKey, enable_range_->getLower());
  pt.set(kEnableOffKey, enable_range_->getUpper());
  pt.set(kKillOnKey, kill_range_->getLower());
  pt.set(kKillOffKey, kill_range_->getUpper());
  pt.set(kModeAcrobatKey, mode_range_->getUpper());
  pt.set(kModeStabilizeKey, mode_range_->getMiddle());
  pt.set(kModeLoiterKey, mode_range_->getLower());
  pt.set(kSubModeOnKey, sub_mode_range_->getLower());
  pt.set(kSubModeOffKey, sub_mode_range_->getUpper());
  pt.set(kGPSw1OnKey, gpsw1_range_->getLower());
  pt.set(kGPSw1OffKey, gpsw1_range_->getUpper());
  pt.set(kGPSw2OnKey, gpsw2_range_->getLower());
  pt.set(kGPSw2OffKey, gpsw2_range_->getUpper());
  if (!pt.save())
  {
    qt::qErrorBox(this, "Failed to save calibration results on GCS.");
    return false;
  }

  return true;
}

bool RCInputCalibrationWidget::saveParamsFC()
{
  const auto req = std::make_shared<tobas_real_msgs::srv::SetRCInputParams::Request>();
  req->roll_left = roll_range_->getLower();
  req->roll_right = roll_range_->getUpper();
  req->pitch_up = pitch_range_->getLower();
  req->pitch_down = pitch_range_->getUpper();
  req->yaw_left = yaw_range_->getLower();
  req->yaw_right = yaw_range_->getUpper();
  req->throttle_up = throt_range_->getLower();
  req->throttle_down = throt_range_->getUpper();
  req->enable_on = enable_range_->getLower();
  req->enable_off = enable_range_->getUpper();
  req->kill_on = kill_range_->getLower();
  req->kill_off = kill_range_->getUpper();
  req->mode_acrobat = mode_range_->getUpper();
  req->mode_stabilize = mode_range_->getMiddle();
  req->mode_loiter = mode_range_->getLower();
  req->sub_mode_on = sub_mode_range_->getLower();
  req->sub_mode_off = sub_mode_range_->getUpper();
  req->gpsw1_on = gpsw1_range_->getLower();
  req->gpsw1_off = gpsw1_range_->getUpper();
  req->gpsw2_on = gpsw2_range_->getLower();
  req->gpsw2_off = gpsw2_range_->getUpper();

  ros2::SyncServiceClient<tobas_real_msgs::srv::SetRCInputParams> sc(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, kSetParamSrv));
  if (!sc.call(req, kSetParamTimeout))
  {
    qt::qErrorBox(this, "Failed to send calibration results to FC.");
    return false;
  }

  const auto res = sc.getResponse();
  if (!res->success)
  {
    qt::qErrorBox(this, "Calibration results are rejected: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void RCInputCalibrationWidget::sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus)
{
  if (!rate_.update(sbus->header.stamp))
    return;

  roll_range_->setValue(sbus->data[real::kRcChannelRoll]);
  pitch_range_->setValue(sbus->data[real::kRcChannelPitch]);
  yaw_range_->setValue(sbus->data[real::kRcChannelYaw]);
  throt_range_->setValue(sbus->data[real::kRcChannelThrot]);
  enable_range_->setValue(sbus->data[real::kRcChannelEnable]);
  kill_range_->setValue(sbus->data[real::kRcChannelKill]);
  mode_range_->setValue(sbus->data[real::kRcChannelMode]);
  sub_mode_range_->setValue(sbus->data[real::kRcChannelSubMode]);
  gpsw1_range_->setValue(sbus->data[real::kRcChannelGPSw1]);
  gpsw2_range_->setValue(sbus->data[real::kRcChannelGPSw2]);
}

void RCInputCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RCInputCalibrationWidget::onStartButtonClicked()
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

  // 一時的にSBUSトピックを購読開始
  sbus_sub_ =
    ros2::createSubscriber(node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, tobas::kSBUSTopic), &self::sbusCb, this);

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
  // メッセージの受信を確認
  if (!roll_range_->hasValue())
  {
    qt::qErrorBox(this, "No S.BUS message is received.");
    reset();
    return;
  }

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
  if (enable_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Enable channel is too narrow.");
    reset();
    return;
  }
  if (kill_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Kill channel is too narrow.");
    reset();
    return;
  }
  if (mode_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Mode channel is too narrow.");
    reset();
    return;
  }
  if (sub_mode_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of Sub-Mode channel is too narrow.");
    reset();
    return;
  }
  if (gpsw1_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of GPSw 1 channel is too narrow.");
    reset();
    return;
  }
  if (gpsw2_range_->getRange() < kMinSignalRange)
  {
    qt::qErrorBox(this, "The signal range of GPSw 2 channel is too narrow.");
    reset();
    return;
  }

  if (!saveParamsGCS())
    return;
  if (!saveParamsFC())
    return;

  qt::qInfoBox(this, "Radio calibration finished successfully.");
  reset();
}
}  // namespace hw
}  // namespace gui
