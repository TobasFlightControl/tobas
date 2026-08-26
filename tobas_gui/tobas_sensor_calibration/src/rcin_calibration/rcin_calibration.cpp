// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/rcin_calibration/rcin_calibration.hpp"

#include <QDebug>

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
RCInputCalibrationWidget::RCInputCalibrationWidget(const rqt::RosQtBridge& bridge, const Drone& drone) : drone_(drone)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Click \"Start\" to begin displaying S.BUS data in the view.\n\n"
    "2. For each channel, operate the stick or switch to ensure it covers the entire range. "
    "If the stick's movement is opposite to that of the bar, adjust the transmitter settings accordingly.\n\n"
    "3. When the full range of all channels has been covered, click \"Finish.\"\n\n",
    cmn::kBodyPSize);

  const auto button_cols = new QHBoxLayout();

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

  // Sticks
  const auto stick_cols = new QHBoxLayout();

  pitch_range_ = new qt::VPositionBarWidget(kMinPeriod, kMaxPeriod);
  pitch_range_->setFixedWidth(kRangeSideShort);
  stick_cols->addWidget(pitch_range_);

  const auto roll_yaw_rows = new QVBoxLayout();
  stick_cols->addLayout(roll_yaw_rows);

  roll_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  roll_range_->setFixedHeight(kRangeSideShort);
  roll_yaw_rows->addWidget(roll_range_);
  qt::addWidgetCenter(new QLabel(std::format("Roll (CH{})", kRcChannelRoll + 1).c_str()), roll_yaw_rows);

  roll_yaw_rows->addStretch();

  const auto pitch_throt_label_cols = new QHBoxLayout();
  roll_yaw_rows->addLayout(pitch_throt_label_cols);

  const auto pitch_label = new QLabel(std::format("Pitch (CH{})", kRcChannelPitch + 1).c_str());
  pitch_label->setAlignment(Qt::AlignLeft);
  pitch_throt_label_cols->addWidget(pitch_label);

  const auto throt_label = new QLabel(std::format("Throttle (CH{})", kRcChannelThrot + 1).c_str());
  throt_label->setAlignment(Qt::AlignRight);
  pitch_throt_label_cols->addWidget(throt_label);

  roll_yaw_rows->addStretch();

  qt::addWidgetCenter(new QLabel(std::format("Yaw (CH{})", kRcChannelYaw + 1).c_str()), roll_yaw_rows);
  yaw_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  yaw_range_->setFixedHeight(kRangeSideShort);
  roll_yaw_rows->addWidget(yaw_range_);

  throt_range_ = new qt::VPositionBarWidget(kMaxPeriod, kMinPeriod);
  throt_range_->setFixedWidth(kRangeSideShort);
  stick_cols->addWidget(throt_range_);

  // Control Switches
  const auto ctrl_switch_form = new qt::FormLayout();

  mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  mode_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addVAlignedRow(std::format("Mode (CH{})", kRcChannelMode + 1).c_str(), mode_range_);

  ctrl_switch_form->addStretch();

  sub_mode_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  sub_mode_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addVAlignedRow(std::format("Sub Mode (CH{})", kRcChannelSubMode + 1).c_str(), sub_mode_range_);

  ctrl_switch_form->addStretch();

  enable_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  enable_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addVAlignedRow(std::format("Enable (CH{})", kRcChannelEnable + 1).c_str(), enable_range_);

  ctrl_switch_form->addStretch();

  kill_range_ = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
  kill_range_->setFixedHeight(kRangeSideShort);
  ctrl_switch_form->addVAlignedRow(std::format("Kill (CH{})", kRcChannelKill + 1).c_str(), kill_range_);

  // General Purpose Switches
  const auto gpsw_form = new qt::FormLayout();

  for (size_t i = 0; i < kMaxNumOfGpsw; ++i) {
    gpsw_labels_[i] = new QLabel();
    gpsw_ranges_[i] = new qt::HPositionBarWidget(kMinPeriod, kMaxPeriod);
    gpsw_ranges_[i]->setFixedHeight(kRangeSideShort);
    gpsw_form->addVAlignedRow(gpsw_labels_[i], gpsw_ranges_[i]);
    if (i < kMaxNumOfGpsw - 1) {
      gpsw_form->addStretch();
    }
  }

  // Layout
  const auto main_signal_rows = new QVBoxLayout();
  main_signal_rows->addLayout(stick_cols, 10);
  main_signal_rows->addStretch(1);
  main_signal_rows->addLayout(ctrl_switch_form, 10);

  const auto rc_range_cols = new QHBoxLayout();
  rc_range_cols->addLayout(main_signal_rows, 10);
  rc_range_cols->addStretch(1);
  rc_range_cols->addLayout(gpsw_form, 10);

  rows_->addWidget(instruction);
  rows_->addLayout(button_cols);
  rows_->addSpacing(50);
  rows_->addLayout(rc_range_cols);

  // Other connections
  connect(&bridge, &rqt::RosQtBridge::sbusReceived, this, &self::sbusCb, Qt::QueuedConnection);
  connect(&bridge, &rqt::RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);

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
    gpsw_labels_.at(i)->setText(std::format("GPSw{} (CH{})", i + 1, kRcChannelGpsw + i + 1).c_str());
    gpsw_ranges_.at(i)->setEnabled(true);
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    gpsw_labels_.at(i)->setText("Not Registered");
    gpsw_ranges_.at(i)->setEnabled(false);
  }
}

void RCInputCalibrationWidget::clearRosInterfaces()
{
  set_params_sc_.reset();

  ros_initialized_ = false;
}

void RCInputCalibrationWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  const auto set_params_srv = path::join(ns, kRemoteIfaceNS, real::handler::rcin::kSetParamSrv);
  set_params_sc_ =
    std::make_shared<ros2::SyncServiceClient<tobas_real_msgs::srv::SetRcInputParams>>(std::move(node), set_params_srv);

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
  if (!pt.initialize((ros2::expandUser(kConfigDirHome) / real::handler::rcin::kConfigFileName))) {
    qt::qErrorBox(this, "Failed to initialize property tree.");
    return false;
  }

  const auto& section = drone_.name;

  pt.set(section, real::handler::rcin::kRollLeftKey, roll_range_->getLower());
  pt.set(section, real::handler::rcin::kRollRightKey, roll_range_->getUpper());
  pt.set(section, real::handler::rcin::kPitchUpKey, pitch_range_->getLower());
  pt.set(section, real::handler::rcin::kPitchDownKey, pitch_range_->getUpper());
  pt.set(section, real::handler::rcin::kYawLeftKey, yaw_range_->getLower());
  pt.set(section, real::handler::rcin::kYawRightKey, yaw_range_->getUpper());
  pt.set(section, real::handler::rcin::kThrotUpKey, throt_range_->getLower());
  pt.set(section, real::handler::rcin::kThrotDownKey, throt_range_->getUpper());
  pt.set(section, real::handler::rcin::kModeAcrobatKey, mode_range_->getUpper());
  pt.set(section, real::handler::rcin::kModeStabilizeKey, mode_range_->getMiddle());
  pt.set(section, real::handler::rcin::kModeLoiterKey, mode_range_->getLower());
  pt.set(section, real::handler::rcin::kSubModeOnKey, sub_mode_range_->getLower());
  pt.set(section, real::handler::rcin::kSubModeOffKey, sub_mode_range_->getUpper());
  pt.set(section, real::handler::rcin::kEnableOnKey, enable_range_->getLower());
  pt.set(section, real::handler::rcin::kEnableOffKey, enable_range_->getUpper());
  pt.set(section, real::handler::rcin::kKillOnKey, kill_range_->getLower());
  pt.set(section, real::handler::rcin::kKillOffKey, kill_range_->getUpper());

  std::array<int, kMaxNumOfGpsw> gpsw_on, gpsw_off;
  for (size_t i = 0; i < numOfGpswChannels(); ++i) {
    gpsw_on[i] = gpsw_ranges_[i]->getLower();
    gpsw_off[i] = gpsw_ranges_[i]->getUpper();
  }
  for (size_t i = numOfGpswChannels(); i < kMaxNumOfGpsw; ++i) {
    gpsw_on[i] = std::numeric_limits<uint16_t>::max();
    gpsw_off[i] = 0;
  }
  pt.set(section, real::handler::rcin::kGpswOnKey, gpsw_on);
  pt.set(section, real::handler::rcin::kGpswOffKey, gpsw_off);

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
