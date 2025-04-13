#include <QDebug>

#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_real_msgs/srv/set_rc_input_params.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "tobas_hardware_setup/rcin_calibration/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

using namespace std;
using namespace real::handler::rcin;

namespace gui
{
namespace hw
{
RCInputCalibrationWidget::RCInputCalibrationWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone), rate_(kTopicRate)
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

  const auto main_signal_rows = new QVBoxLayout();
  rc_range_cols->addLayout(main_signal_rows, 1);

  // Sticks
  const auto stick_cols = new QHBoxLayout();
  main_signal_rows->addLayout(stick_cols, 1);

  pitch_range_ = new qt::VPositionBarWidget(kMinPeriod, kMaxPeriod);
  pitch_range_->setFixedWidth(kRangeSideShort);
  stick_cols->addWidget(pitch_range_);

  const auto roll_yaw_rows = new QVBoxLayout();
  stick_cols->addLayout(roll_yaw_rows);

  roll_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  roll_range_->setFixedHeight(kRangeSideShort);
  roll_yaw_rows->addWidget(roll_range_);
  qt::addWidgetCenter(new QLabel(format("Roll (CH{})", real::kRcChannelRoll + 1).c_str()), roll_yaw_rows);

  roll_yaw_rows->addStretch();

  const auto pitch_throt_label_cols = new QHBoxLayout();
  roll_yaw_rows->addLayout(pitch_throt_label_cols);

  const auto pitch_label = new QLabel(format("Pitch (CH{})", real::kRcChannelPitch + 1).c_str());
  pitch_label->setAlignment(Qt::AlignLeft);
  pitch_throt_label_cols->addWidget(pitch_label);

  const auto throt_label = new QLabel(format("Throttle (CH{})", real::kRcChannelThrot + 1).c_str());
  throt_label->setAlignment(Qt::AlignRight);
  pitch_throt_label_cols->addWidget(throt_label);

  roll_yaw_rows->addStretch();

  qt::addWidgetCenter(new QLabel(format("Yaw (CH{})", real::kRcChannelYaw + 1).c_str()), roll_yaw_rows);
  yaw_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  yaw_range_->setFixedHeight(kRangeSideShort);
  roll_yaw_rows->addWidget(yaw_range_);

  throt_range_ = new qt::VPositionBarWidget(kMinPeriod, kMaxPeriod);
  throt_range_->setFixedWidth(kRangeSideShort);
  stick_cols->addWidget(throt_range_);

  // Control Switches
  const auto ctrl_switch_form = new qt::FormLayout();
  main_signal_rows->addLayout(ctrl_switch_form, 1);

  enable_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  enable_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addRow(format("Enable (CH{})", real::kRcChannelEnable + 1).c_str(), enable_range_);

  ctrl_switch_form->addStretch();

  kill_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  kill_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addRow(format("Kill (CH{})", real::kRcChannelKill + 1).c_str(), kill_range_);

  ctrl_switch_form->addStretch();

  mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  mode_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addRow(format("Mode (CH{})", real::kRcChannelMode + 1).c_str(), mode_range_);

  ctrl_switch_form->addStretch();

  sub_mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  sub_mode_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addRow(format("Sub Mode (CH{})", real::kRcChannelSubMode + 1).c_str(), sub_mode_range_);

  // General Purpose Switches
  const auto gpsw_form = new qt::FormLayout();
  rc_range_cols->addLayout(gpsw_form, 1);

  for (size_t i = 0; i < tobas::kMaxNumOfGpsw; ++i)
  {
    gpsw_ranges_[i] = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
    gpsw_ranges_[i]->setFixedHeight(kRangeSideShort);
    gpsw_form->addRow(format("GPSw{} (CH{})", i + 1, real::kRcChannelGpsw + i).c_str(), gpsw_ranges_[i]);
    if (i < tobas::kMaxNumOfGpsw - 1)
      gpsw_form->addStretch();
  }

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
  if (sbus_sub_)
    sbus_sub_.reset();

  rate_.reset();

  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();

  enable_range_->clear();
  kill_range_->clear();
  mode_range_->clear();
  sub_mode_range_->clear();

  enable_range_->setText(kOnOffText);
  kill_range_->setText(kOnOffText);
  mode_range_->setText(kModeText);
  sub_mode_range_->setText(kOnOffText);

  for (auto& gpsw_range : gpsw_ranges_)
  {
    gpsw_range->clear();
    gpsw_range->setText(kOnOffText);
  }

  start_button_->setEnabled(true);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);
}

void RCInputCalibrationWidget::updateInternalDataStructures()
{
  reset();

  arming_.reset();
  arming_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);

  setEnabled(true);
}

size_t RCInputCalibrationWidget::numOfGpswChannels() const
{
  if (drone_.num_sbus_channles < tobas::kMinSbusChannels)
  {
    qWarning() << "The number of S.BUS channels cannot be lower than " << tobas::kMinSbusChannels << ".";
    return 0;
  }
  else if (drone_.num_sbus_channles > tobas::kMaxSbusChannels)
  {
    qWarning() << "The number of S.BUS channels cannot be greater than " << tobas::kMinSbusChannels << ".";
    return tobas::kMaxNumOfGpsw;
  }

  return drone_.num_sbus_channles - tobas::kMinSbusChannels;
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

  array<int, tobas::kMaxNumOfGpsw> gpsw_on, gpsw_off;
  for (size_t i = 0; i < numOfGpswChannels(); ++i)
  {
    gpsw_on[i] = gpsw_ranges_[i]->getLower();
    gpsw_off[i] = gpsw_ranges_[i]->getUpper();
  }
  for (size_t i = numOfGpswChannels(); tobas::kMaxSbusChannels; ++i)
  {
    gpsw_on[i] = numeric_limits<uint16_t>::max();
    gpsw_off[i] = 0;
  }
  pt.set(kGpswOnKey, gpsw_on);
  pt.set(kGpswOffKey, gpsw_off);

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

  for (size_t i = 0; i < numOfGpswChannels(); ++i)
  {
    req->gpsw_on[i] = gpsw_ranges_[i]->getLower();
    req->gpsw_off[i] = gpsw_ranges_[i]->getUpper();
  }
  for (size_t i = numOfGpswChannels(); tobas::kMaxSbusChannels; ++i)
  {
    req->gpsw_on[i] = numeric_limits<uint16_t>::max();
    req->gpsw_off[i] = 0;
  }

  ros2::SyncServiceClient<tobas_real_msgs::srv::SetRCInputParams> sc(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, kSetParamSrv));
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

  for (size_t i = 0; i < tobas::kMaxNumOfGpsw; ++i)
    gpsw_ranges_[i]->setValue(sbus->data[real::kRcChannelGpsw + i]);
}

void RCInputCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void RCInputCalibrationWidget::onStartButtonClicked()
{
  // アームされていないことを確認
  if (!arming_)
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
  sbus_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kSbusTopic), &self::sbusCb, this);

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

  for (size_t i = 0; i < numOfGpswChannels(); ++i)
  {
    if (gpsw_ranges_.at(i)->getValue() < kMinSignalRange)
    {
      qt::qErrorBox(this, "The signal range of GPSw " + QString::number(i + 1) + " channel is too narrow.");
      reset();
      return;
    }
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
