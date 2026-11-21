// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/rcin_calibration/rcin_calibration.hpp"

#include <QDebug>
#include <QGridLayout>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_real_common/handler.hpp>
#include <tobas_ros2_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
RCInputCalibrationWidget::RCInputCalibrationWidget(
  rclcpp::Node::SharedPtr node,
  const RosQtBridge& bridge,
  const Drone& drone)
  : node_(node), drone_(drone)
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

  roll_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  pitch_range_ = new qt::VPositionBarWidget(kMinPeriod, kMaxPeriod);
  yaw_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  throt_range_ = new qt::VPositionBarWidget(kMaxPeriod, kMinPeriod);
  mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  sub_mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  enable_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  kill_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);

  for (size_t i = 0; i < kMaxNumOfGpsw; ++i) {
    gpsw_labels_[i] = new QLabel("Not Registered");
    gpsw_ranges_[i] = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  }

  // Size
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  finish_button_->setFixedSize(kButtonWidth, kButtonHeight);
  cancel_button_->setFixedSize(kButtonWidth, kButtonHeight);

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

  const auto root_rows = rows_;
  root_rows->addWidget(instruction, 0);
  root_rows->addLayout(button_cols, 0);
  root_rows->addSpacing(40);
  root_rows->addLayout(rc_range_grid, 1);

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(finish_button_, &QPushButton::clicked, this, &self::onFinishButtonClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  connect(&bridge, &RosQtBridge::sbusReceived, this, &self::sbusCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);

  reset();
}

const char* RCInputCalibrationWidget::title() const
{
  return "Calibrate RC Input";
}

void RCInputCalibrationWidget::reset()
{
  running_ = false;

  sbus_.reset();
  arming_.reset();

  start_button_->setEnabled(true);
  finish_button_->setEnabled(false);
  cancel_button_->setEnabled(false);

  roll_range_->clear();
  pitch_range_->clear();
  yaw_range_->clear();
  throt_range_->clear();

  mode_range_->clear();
  sub_mode_range_->clear();
  enable_range_->clear();
  kill_range_->clear();

  mode_range_->setLowerText("Loiter");
  mode_range_->setCenterText("Stabilize");
  mode_range_->setUpperText("Acrobat");

  sub_mode_range_->setLowerText(kOnText);
  sub_mode_range_->setUpperText(kOffText);

  enable_range_->setLowerText(kOnText);
  enable_range_->setUpperText(kOffText);

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
  reset();

  const auto set_params_srv = path::join('/', drone_.name, kRemoteIfaceNS, real::handler::rcin::kSetParamSrv);
  set_params_sc_ =
    std::make_shared<ros2::SyncServiceClient<tobas_real_msgs::srv::SetRcInputParams>>(node_, set_params_srv);

  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    gpsw_labels_.at(i)->setText(std::format("GPSw{} (CH{})", i + 1, kRcChannelGpsw + i + 1).c_str());
    gpsw_ranges_.at(i)->setEnabled(true);
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    gpsw_labels_.at(i)->setText("Not Registered");
    gpsw_ranges_.at(i)->setEnabled(false);
  }
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
  if (!pt.initialize((ros2::expandUser(kConfigDirHome) / real::handler::rcin::kConfigFileName))) {
    qt::qErrorBox(this, "Failed to initialize property tree.");
    return false;
  }

  const auto ns = '/' + drone_.name;

  pt.set(ns, real::handler::rcin::kRollLeftKey, roll_range_->getLower());
  pt.set(ns, real::handler::rcin::kRollRightKey, roll_range_->getUpper());
  pt.set(ns, real::handler::rcin::kPitchUpKey, pitch_range_->getLower());
  pt.set(ns, real::handler::rcin::kPitchDownKey, pitch_range_->getUpper());
  pt.set(ns, real::handler::rcin::kYawLeftKey, yaw_range_->getLower());
  pt.set(ns, real::handler::rcin::kYawRightKey, yaw_range_->getUpper());
  pt.set(ns, real::handler::rcin::kThrotUpKey, throt_range_->getLower());
  pt.set(ns, real::handler::rcin::kThrotDownKey, throt_range_->getUpper());

  pt.set(ns, real::handler::rcin::kModeAcrobatKey, mode_range_->getUpper());
  pt.set(ns, real::handler::rcin::kModeStabilizeKey, mode_range_->getMiddle());
  pt.set(ns, real::handler::rcin::kModeLoiterKey, mode_range_->getLower());
  pt.set(ns, real::handler::rcin::kSubModeOnKey, sub_mode_range_->getLower());
  pt.set(ns, real::handler::rcin::kSubModeOffKey, sub_mode_range_->getUpper());
  pt.set(ns, real::handler::rcin::kEnableOnKey, enable_range_->getLower());
  pt.set(ns, real::handler::rcin::kEnableOffKey, enable_range_->getUpper());
  pt.set(ns, real::handler::rcin::kKillOnKey, kill_range_->getLower());
  pt.set(ns, real::handler::rcin::kKillOffKey, kill_range_->getUpper());

  std::array<int, kMaxNumOfGpsw> gpsw_on, gpsw_off;
  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    gpsw_on[i] = gpsw_ranges_[i]->getLower();
    gpsw_off[i] = gpsw_ranges_[i]->getUpper();
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    gpsw_on[i] = std::numeric_limits<uint16_t>::max();
    gpsw_off[i] = 0;
  }
  pt.set(ns, real::handler::rcin::kGpswOnKey, gpsw_on);
  pt.set(ns, real::handler::rcin::kGpswOffKey, gpsw_off);

  if (!pt.save()) {
    qt::qErrorBox(this, "Failed to save calibration results on GCS.");
    return false;
  }

  return true;
}

bool RCInputCalibrationWidget::saveParamsToFc()
{
  const auto req = std::make_shared<tobas_real_msgs::srv::SetRcInputParams::Request>();

  req->roll_left = roll_range_->getLower();
  req->roll_right = roll_range_->getUpper();
  req->pitch_up = pitch_range_->getLower();
  req->pitch_down = pitch_range_->getUpper();
  req->yaw_left = yaw_range_->getLower();
  req->yaw_right = yaw_range_->getUpper();
  req->throttle_up = throt_range_->getLower();
  req->throttle_down = throt_range_->getUpper();

  req->mode_acrobat = mode_range_->getUpper();
  req->mode_stabilize = mode_range_->getMiddle();
  req->mode_loiter = mode_range_->getLower();
  req->sub_mode_on = sub_mode_range_->getLower();
  req->sub_mode_off = sub_mode_range_->getUpper();
  req->enable_on = enable_range_->getLower();
  req->enable_off = enable_range_->getUpper();
  req->kill_on = kill_range_->getLower();
  req->kill_off = kill_range_->getUpper();

  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    req->gpsw_on[i] = gpsw_ranges_[i]->getLower();
    req->gpsw_off[i] = gpsw_ranges_[i]->getUpper();
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    req->gpsw_on[i] = std::numeric_limits<uint16_t>::max();
    req->gpsw_off[i] = 0;
  }

  const auto res = set_params_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to send calibration results to FC.");
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
  qt::qInfoBox(this, "Radio calibration was canceled.");
  reset();
}

void RCInputCalibrationWidget::onFinishButtonClicked()
{
  // Confirm that messages have been received.
  if (!roll_range_->hasValue()) {
    qt::qWarnBox(this, "No S.BUS message is received.");
    return;
  }

  // Check each channel value range.
  if (roll_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Roll channel is too narrow.");
    return;
  }
  if (pitch_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Pitch channel is too narrow.");
    return;
  }
  if (yaw_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Yaw channel is too narrow.");
    return;
  }
  if (throt_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Throttle channel is too narrow.");
    return;
  }
  if (mode_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Mode channel is too narrow.");
    return;
  }
  if (sub_mode_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Sub-Mode channel is too narrow.");
    return;
  }
  if (enable_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Enable channel is too narrow.");
    return;
  }
  if (kill_range_->getRange() < kMinSignalRange) {
    qt::qWarnBox(this, "The signal range of Kill channel is too narrow.");
    return;
  }

  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    if (gpsw_ranges_.at(i)->getRange() < kMinSignalRange) {
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

void RCInputCalibrationWidget::sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus)
{
  if (sbus->frame_lost) {
    return;
  }

  sbus_ = sbus;

  if (!running_) {
    return;
  }

  roll_range_->setValue(sbus->periods.at(kRcChannelRoll));
  pitch_range_->setValue(sbus->periods.at(kRcChannelPitch));
  yaw_range_->setValue(sbus->periods.at(kRcChannelYaw));
  throt_range_->setValue(sbus->periods.at(kRcChannelThrot));

  mode_range_->setValue(sbus->periods.at(kRcChannelMode));
  sub_mode_range_->setValue(sbus->periods.at(kRcChannelSubMode));
  enable_range_->setValue(sbus->periods.at(kRcChannelEnable));
  kill_range_->setValue(sbus->periods.at(kRcChannelKill));

  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    gpsw_ranges_[i]->setValue(sbus->periods.at(kRcChannelGpsw + i));
  }
}

void RCInputCalibrationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (running_ && arming->data) {
    reset();
    qt::qWarnBox(this, "Radio calibration was canceled because an arming command was issued.");
  }

  arming_ = arming;
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
