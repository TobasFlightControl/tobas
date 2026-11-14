// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/rcin_calibration/rcin_calibration.hpp"

#include <QDebug>
#include <QGridLayout>
#include <QString>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_ros2_tools/util.hpp>

#include "tobas_sensor_calibration/constants.hpp"

namespace tobas::gui::sc
{
namespace
{
constexpr char kNotRegisteredText[] = "Not Registered";
}  // namespace

RCInputCalibrationWidget::RCInputCalibrationWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone)
  : drone_(_drone)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Click 'Start' to begin displaying S.BUS data in the view.\n"
    "2. For each channel, operate the stick or switch to ensure it covers the entire range. "
    "If the stick's movement is opposite to that of the bar, adjust the transmitter settings accordingly.\n"
    "3. When the full range of all channels has been covered, click 'Finish.'\n");

  start_button_ = new QPushButton("Start");
  finish_button_ = new QPushButton("Finish");
  cancel_button_ = new QPushButton("Cancel");

  const auto roll_label = new QLabel(QString("Roll (CH%1)").arg(kRcChannelRoll + 1));
  const auto pitch_label = new QLabel(QString("Pitch (CH%1)").arg(kRcChannelPitch + 1));
  const auto yaw_label = new QLabel(QString("Yaw (CH%1)").arg(kRcChannelYaw + 1));
  const auto throt_label = new QLabel(QString("Throttle (CH%1)").arg(kRcChannelThrot + 1));
  const auto mode_label = new QLabel(QString("Mode (CH%1)").arg(kRcChannelMode + 1));
  const auto sub_mode_label = new QLabel(QString("Sub Mode (CH%1)").arg(kRcChannelSubMode + 1));
  const auto enable_label = new QLabel(QString("Enable (CH%1)").arg(kRcChannelEnable + 1));
  const auto kill_label = new QLabel(QString("Kill (CH%1)").arg(kRcChannelKill + 1));

  constexpr int kMinPeriod = 0;
  constexpr int kMaxPeriod = 2000;
  roll_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  pitch_range_ = new qt::VPositionBarWidget(kMinPeriod, kMaxPeriod);
  yaw_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  throt_range_ = new qt::VPositionBarWidget(kMaxPeriod, kMinPeriod);
  mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  sub_mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  enable_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  kill_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);

  for (size_t i = 0; i < kMaxNumOfGpsw; ++i) {
    gpsw_labels_[i] = new QLabel(kNotRegisteredText);
    gpsw_ranges_[i] = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  }

  reset();

  // Size
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  finish_button_->setFixedSize(kButtonWidth, kButtonHeight);
  cancel_button_->setFixedSize(kButtonWidth, kButtonHeight);

  constexpr int kRangeSideShort = 48;
  roll_range_->setFixedHeight(kRangeSideShort);
  pitch_range_->setFixedWidth(kRangeSideShort);
  yaw_range_->setFixedHeight(kRangeSideShort);
  throt_range_->setFixedWidth(kRangeSideShort);
  mode_range_->setFixedHeight(kRangeSideShort);
  sub_mode_range_->setFixedHeight(kRangeSideShort);
  enable_range_->setFixedHeight(kRangeSideShort);
  kill_range_->setFixedHeight(kRangeSideShort);
  for (const auto& range : gpsw_ranges_) {
    range->setFixedHeight(kRangeSideShort);
  }

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(start_button_);
  button_cols->addWidget(finish_button_);
  button_cols->addWidget(cancel_button_);
  button_cols->addStretch(1);

  const auto stick_grid = new QGridLayout();
  stick_grid->setColumnStretch(1, 1);
  stick_grid->setColumnStretch(2, 1);
  stick_grid->setRowStretch(2, 1);
  stick_grid->addWidget(pitch_range_, 0, 0, 5, 1);
  stick_grid->addWidget(roll_range_, 0, 1, 1, 2);
  stick_grid->addWidget(roll_label, 1, 1, 1, 2, Qt::AlignCenter);
  stick_grid->addWidget(pitch_label, 2, 1, Qt::AlignLeft | Qt::AlignVCenter);
  stick_grid->addWidget(throt_label, 2, 2, Qt::AlignRight | Qt::AlignVCenter);
  stick_grid->addWidget(yaw_label, 3, 1, 1, 2, Qt::AlignCenter);
  stick_grid->addWidget(yaw_range_, 4, 1, 1, 2);
  stick_grid->addWidget(throt_range_, 0, 3, 5, 1);

  const auto rc_range_grid = new QGridLayout();
  rc_range_grid->setColumnStretch(1, 1);
  rc_range_grid->setColumnMinimumWidth(2, 40);
  rc_range_grid->setColumnStretch(4, 1);
  rc_range_grid->addLayout(stick_grid, 0, 0, 7, 2);
  rc_range_grid->addWidget(mode_label, 8, 0, Qt::AlignVCenter);
  rc_range_grid->addWidget(mode_range_, 8, 1, Qt::AlignVCenter);
  rc_range_grid->addWidget(sub_mode_label, 10, 0, Qt::AlignVCenter);
  rc_range_grid->addWidget(sub_mode_range_, 10, 1, Qt::AlignVCenter);
  rc_range_grid->addWidget(enable_label, 12, 0, Qt::AlignVCenter);
  rc_range_grid->addWidget(enable_range_, 12, 1, Qt::AlignVCenter);
  rc_range_grid->addWidget(kill_label, 14, 0, Qt::AlignVCenter);
  rc_range_grid->addWidget(kill_range_, 14, 1, Qt::AlignVCenter);

  for (size_t i = 0; i < kMaxNumOfGpsw; ++i) {
    const auto row = static_cast<int>(2 * i);
    rc_range_grid->addWidget(gpsw_labels_[i], row, 3, Qt::AlignVCenter);
    rc_range_grid->addWidget(gpsw_ranges_[i], row, 4, Qt::AlignVCenter);
    if (i + 1 < kMaxNumOfGpsw) {
      rc_range_grid->setRowStretch(row + 1, 1);
    }
  }

  rows_->addWidget(instruction, 0);
  rows_->addLayout(button_cols, 0);
  rows_->addSpacing(40);
  rows_->addLayout(rc_range_grid, 1);

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(finish_button_, &QPushButton::clicked, this, &self::onFinishButtonClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  connect(&_bridge, &rqt::RosQtBridge::sbusReceived, this, &self::sbusCb, Qt::QueuedConnection);
  connect(&_bridge, &rqt::RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

const char* RCInputCalibrationWidget::title() const
{
  return "Calibrate RC Input";
}

void RCInputCalibrationWidget::reset()
{
  constexpr char kOnText[] = "ON";
  constexpr char kOffText[] = "OFF";

  running_ = false;

  sbus_.reset();
  arming_.reset();

  start_button_->setEnabled(ros_initialized_);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);

  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();

  mode_range_->clear();
  mode_range_->setLowerText("Loiter");
  mode_range_->setCenterText("Stabilize");
  mode_range_->setUpperText("Acrobat");

  sub_mode_range_->clear();
  sub_mode_range_->setLowerText(kOnText);
  sub_mode_range_->setUpperText(kOffText);

  enable_range_->clear();
  enable_range_->setLowerText(kOnText);
  enable_range_->setUpperText(kOffText);

  kill_range_->clear();
  kill_range_->setLowerText(kOnText);
  kill_range_->setUpperText(kOffText);

  for (auto& gpsw_range : gpsw_ranges_) {
    gpsw_range->clear();
    gpsw_range->setLowerText(kOnText);
    gpsw_range->setUpperText(kOffText);
  }
}

void RCInputCalibrationWidget::updateInternalDataStructures()
{
  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    gpsw_labels_.at(i)->setText(QString("GPSw%1 (CH%2)").arg(i + 1).arg(kRcChannelGpsw + i + 1));
    gpsw_ranges_.at(i)->setEnabled(true);
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    gpsw_labels_.at(i)->setText(kNotRegisteredText);
    gpsw_ranges_.at(i)->setEnabled(false);
  }
}

void RCInputCalibrationWidget::clearRosInterfaces()
{
  set_params_sc_.reset();

  ros_initialized_ = false;
}

void RCInputCalibrationWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  const auto set_params_srv = path::join(_ns, kRemoteIfaceNS, real::handler::rcin::kSetParamSrv);
  set_params_sc_.emplace(std::move(_node), set_params_srv);

  ros_initialized_ = true;
}

size_t RCInputCalibrationWidget::numOfGpswChannels() const
{
  if (drone_.num_sbus_channels < kMinSbusChannels) {
    qWarning().nospace() << "The number of S.BUS channels cannot be lower than " << kMinSbusChannels << ".";
    return 0;
  }
  else if (drone_.num_sbus_channels > kMaxSbusChannels) {
    qWarning().nospace() << "The number of S.BUS channels cannot be greater than " << kMinSbusChannels << ".";
    return kMaxNumOfGpsw;
  }

  return drone_.num_sbus_channels - kMinSbusChannels;
}

bool RCInputCalibrationWidget::saveParamsToGcs()
{
  ptree::PropertyTree pt;
  const auto config_path = ros2::expandUser(kConfigDirHome) / real::handler::rcin::kConfigFileName;
  if (const auto result = pt.initialize(config_path); !result) {
    qt::qErrorBox(this, "Failed to initialize property tree: " + QString::fromStdString(result.error()));
    return false;
  }

  const auto& section = drone_.name;

  pt.set(section, real::handler::rcin::kRollLeftKey, roll_range_->getLower().value());
  pt.set(section, real::handler::rcin::kRollRightKey, roll_range_->getUpper().value());
  pt.set(section, real::handler::rcin::kPitchUpKey, pitch_range_->getLower().value());
  pt.set(section, real::handler::rcin::kPitchDownKey, pitch_range_->getUpper().value());
  pt.set(section, real::handler::rcin::kYawLeftKey, yaw_range_->getLower().value());
  pt.set(section, real::handler::rcin::kYawRightKey, yaw_range_->getUpper().value());
  pt.set(section, real::handler::rcin::kThrotUpKey, throt_range_->getLower().value());
  pt.set(section, real::handler::rcin::kThrotDownKey, throt_range_->getUpper().value());
  pt.set(section, real::handler::rcin::kModeAcrobatKey, mode_range_->getUpper().value());
  pt.set(section, real::handler::rcin::kModeStabilizeKey, mode_range_->getMiddle().value());
  pt.set(section, real::handler::rcin::kModeLoiterKey, mode_range_->getLower().value());
  pt.set(section, real::handler::rcin::kSubModeOnKey, sub_mode_range_->getLower().value());
  pt.set(section, real::handler::rcin::kSubModeOffKey, sub_mode_range_->getUpper().value());
  pt.set(section, real::handler::rcin::kEnableOnKey, enable_range_->getLower().value());
  pt.set(section, real::handler::rcin::kEnableOffKey, enable_range_->getUpper().value());
  pt.set(section, real::handler::rcin::kKillOnKey, kill_range_->getLower().value());
  pt.set(section, real::handler::rcin::kKillOffKey, kill_range_->getUpper().value());

  std::array<int, kMaxNumOfGpsw> gpsw_on, gpsw_off;
  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    gpsw_on[i] = gpsw_ranges_[i]->getLower().value();
    gpsw_off[i] = gpsw_ranges_[i]->getUpper().value();
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    gpsw_on[i] = std::numeric_limits<uint16_t>::max();
    gpsw_off[i] = 0;
  }
  pt.set(section, real::handler::rcin::kGpswOnKey, gpsw_on);
  pt.set(section, real::handler::rcin::kGpswOffKey, gpsw_off);

  if (const auto result = pt.save(); !result) {
    qt::qErrorBox(this, "Failed to save calibration results on GCS: " + QString::fromStdString(result.error()));
    return false;
  }

  return true;
}

bool RCInputCalibrationWidget::saveParamsToFc()
{
  const auto req = std::make_shared<tobas_real_msgs::srv::SetRcInputParams::Request>();

  req->roll_left = roll_range_->getLower().value();
  req->roll_right = roll_range_->getUpper().value();
  req->pitch_up = pitch_range_->getLower().value();
  req->pitch_down = pitch_range_->getUpper().value();
  req->yaw_left = yaw_range_->getLower().value();
  req->yaw_right = yaw_range_->getUpper().value();
  req->throttle_up = throt_range_->getLower().value();
  req->throttle_down = throt_range_->getUpper().value();
  req->mode_acrobat = mode_range_->getUpper().value();
  req->mode_stabilize = mode_range_->getMiddle().value();
  req->mode_loiter = mode_range_->getLower().value();
  req->sub_mode_on = sub_mode_range_->getLower().value();
  req->sub_mode_off = sub_mode_range_->getUpper().value();
  req->enable_on = enable_range_->getLower().value();
  req->enable_off = enable_range_->getUpper().value();
  req->kill_on = kill_range_->getLower().value();
  req->kill_off = kill_range_->getUpper().value();

  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    req->gpsw_on[i] = gpsw_ranges_[i]->getLower().value();
    req->gpsw_off[i] = gpsw_ranges_[i]->getUpper().value();
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    req->gpsw_on[i] = std::numeric_limits<uint16_t>::max();
    req->gpsw_off[i] = 0;
  }

  qInfo() << "Sending radio calibration results to the flight controller.";
  qt::WaitSpinnerWidget spinner(Qt::WindowModal, this);
  spinner.start();
  const auto res = set_params_sc_->sendRequestAndWait(req);
  spinner.stop();

  if (!res) {
    qt::qErrorBox(this, "Failed to send calibration results to the flight controller.");
    return false;
  }

  if (!res->success) {
    qt::qErrorBox(this, "Calibration results are rejected: " + QString::fromStdString(res->message));
    return false;
  }

  return true;
}

void RCInputCalibrationWidget::onStartButtonClicked()
{
  qDebug() << "RCInputCalibrationWidget::onStartButtonClicked";

  // Confirm that the vehicle is not armed.
  if (!arming_) {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status has not been received yet.");
    return;
  }
  if (arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
    return;
  }

  // Confirm that required topics have been received.
  if (!sbus_) {
    qt::qWarnBox(this, "S.BUS has not been received yet.");
    return;
  }

  start_button_->setEnabled(false);
  finish_button_->setEnabled(true);
  cancel_button_->setEnabled(true);

  running_ = true;
  qt::qInfoBox(this, "Radio calibration started.");
}

void RCInputCalibrationWidget::onCancelButtonClicked()
{
  qDebug() << "RCInputCalibrationWidget::onCancelButtonClicked";

  qt::qInfoBox(this, "Radio calibration was canceled.");
  reset();
}

void RCInputCalibrationWidget::onFinishButtonClicked()
{
  qDebug() << "RCInputCalibrationWidget::onFinishButtonClicked";

  // Each S.BUS channel value is at least 1000 and at most 2000,
  // so using 1000 as the threshold prevents the human error of moving a three-position switch through only two positions.
  constexpr int kMinSignalRange = 1000;

  // Confirm that messages have been received.
  if (!roll_range_->getValue()) {
    qt::qWarnBox(this, "No S.BUS message is received.");
    return;
  }

  // Check each channel value range.
  if (roll_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Roll channel is too narrow.");
    return;
  }
  if (pitch_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Pitch channel is too narrow.");
    return;
  }
  if (yaw_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Yaw channel is too narrow.");
    return;
  }
  if (throt_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Throttle channel is too narrow.");
    return;
  }
  if (mode_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Mode channel is too narrow.");
    return;
  }
  if (sub_mode_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Sub-Mode channel is too narrow.");
    return;
  }
  if (enable_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Enable channel is too narrow.");
    return;
  }
  if (kill_range_->getRange().value() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Kill channel is too narrow.");
    return;
  }

  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    if (gpsw_ranges_.at(i)->getRange().value() < kMinSignalRange) {
      qt::qWarnBox(this, "The signal range of GPSw " + QString::number(i + 1) + " channel is too narrow.");
      return;
    }
  }

  if (!saveParamsToGcs()) {
    return;
  }
  if (!saveParamsToFc()) {
    return;
  }

  reset();
  qt::qInfoBox(this, "Radio calibration finished successfully.");
}

void RCInputCalibrationWidget::sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& _sbus)
{
  if (_sbus->frame_lost) {
    return;
  }

  sbus_ = _sbus;

  roll_range_->setValue(_sbus->periods.at(kRcChannelRoll));
  pitch_range_->setValue(_sbus->periods.at(kRcChannelPitch));
  yaw_range_->setValue(_sbus->periods.at(kRcChannelYaw));
  throt_range_->setValue(_sbus->periods.at(kRcChannelThrot));
  mode_range_->setValue(_sbus->periods.at(kRcChannelMode));
  sub_mode_range_->setValue(_sbus->periods.at(kRcChannelSubMode));
  enable_range_->setValue(_sbus->periods.at(kRcChannelEnable));
  kill_range_->setValue(_sbus->periods.at(kRcChannelKill));
  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    gpsw_ranges_[i]->setValue(_sbus->periods.at(kRcChannelGpsw + i));
  }

  if (running_) {
    roll_range_->updateRangeFromValue();
    pitch_range_->updateRangeFromValue();
    yaw_range_->updateRangeFromValue();
    throt_range_->updateRangeFromValue();
    mode_range_->updateRangeFromValue();
    sub_mode_range_->updateRangeFromValue();
    enable_range_->updateRangeFromValue();
    kill_range_->updateRangeFromValue();
    for (size_t i = 0; i < numOfGpswChannels(); ++i) {
      gpsw_ranges_[i]->updateRangeFromValue();
    }
  }
}

void RCInputCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming)
{
  if (running_ && _arming->data) {
    reset();
    qt::qWarnBox(this, "Radio calibration was canceled because an arming command was issued.");
  }

  arming_ = _arming;
}
}  // namespace tobas::gui::sc
