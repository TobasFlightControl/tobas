// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/rotor_test/rotor_test.hpp"

#include <ranges>

#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_constants/node.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_math/core.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_yaml_tools/core.hpp>

using namespace std::chrono_literals;
namespace tobas
{
namespace gui
{
namespace at
{
namespace
{
std::string paramName(size_t ch)
{
  return param::kRpmControlGainPrefix + std::to_string(ch);
}
}  // namespace

RotorTestWidget::RotorTestWidget(const rqt::RosQtBridge& bridge, const Drone& drone) : bridge_(bridge), drone_(drone)
{
  registered_.fill(false);

  const auto warning =
    new qt::DescriptionWidget("Warning: Ensure that propellers are removed from motors.\n\n", cmn::kBodyPSize);
  warning->setStyleSheet("color: red; font-weight: bold;");
  rows_->addWidget(warning);

  const auto instruction = new qt::DescriptionWidget(
    "1. Connect the ESCs to the FC in the correct order.\n\n"
    "2. Click \"Start\" to enable motors.\n\n"
    "3. For each channel, confirm the following:\n"
    "   - The motor rotates in the correct direction. If not, swap any two of the three ESC-motor connections.\n"
    "   - The motor does not rotate when the command RPM is 0.\n\n"
    "4. Tune the control gain of each channel to the maximum value at which no vibrations or abnormal noise occur.\n\n"
    "5. Click \"Save\" to save the control gains.\n\n"
    "6. Click \"Stop\" to disable motors.\n\n",
    cmn::kBodyPSize);
  rows_->addWidget(instruction);

  const auto button_cols = new QHBoxLayout();
  rows_->addLayout(button_cols);

  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);
  start_button_->setEnabled(false);
  button_cols->addWidget(start_button_);
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);

  stop_button_ = new QPushButton("Stop");
  stop_button_->setFixedSize(kButtonWidth, kButtonHeight);
  stop_button_->setEnabled(false);
  button_cols->addWidget(stop_button_);
  connect(stop_button_, &QPushButton::clicked, this, &self::onStopButtonClicked);

  save_button_ = new QPushButton("Save");
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  save_button_->setEnabled(false);
  button_cols->addWidget(save_button_);
  connect(save_button_, &QPushButton::clicked, this, &self::onSaveButtonClicked);

  button_cols->addStretch();

  const auto rotor_cols = new QHBoxLayout();
  rows_->addLayout(rotor_cols);

  for (size_t ch = 0; ch < kMaxDshotChannels; ++ch) {
    rotor_widgets_.at(ch) = new RotorWidget();
    rotor_cols->addWidget(rotor_widgets_.at(ch));
    connect(
      rotor_widgets_.at(ch),
      &RotorWidget::targetRPMChanged,
      std::bind(&self::onTargetRPMChanged, this, std::placeholders::_1, ch));
    connect(
      rotor_widgets_.at(ch),
      &RotorWidget::gainChanged,
      std::bind(&self::onGainChanged, this, std::placeholders::_1, ch));
  }

  connect(&update_timer_, &QTimer::timeout, this, &self::onUpdateTimerTimeout);
  connect(&bridge, &rqt::RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

const char* RotorTestWidget::title() const
{
  return "Test Propulsion Rotors";
}

void RotorTestWidget::reset()
{
  disconnect(rotor_states_conn_);

  // Disable motor widgets.
  for (const auto& widget : rotor_widgets_) {
    widget->reset();
    widget->setEnabled(false);
  }

  // Stop the timer.
  update_timer_.stop();

  start_button_->setEnabled(numRegisteredChannels() > 0 && node_);
  stop_button_->setEnabled(false);
  save_button_->setEnabled(false);

  running_ = false;
  arming_.reset();
}

void RotorTestWidget::updateProject(const QString& proj_path)
{
  // Update the project path.
  proj_paths_.setProjPath(proj_path);

  // Initialize.
  registered_.fill(false);
  for (size_t ch = 0; ch < kMaxDshotChannels; ++ch) {
    const auto text = "CH" + QString::number(ch) + ": unregistered";
    rotor_widgets_.at(ch)->setText(text);
  }

  if (drone_.prop->type() == PropulsionSystem::kElectric) {
    eprop_ = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone_.prop);

    // Register motors.
    for (const auto& [link_name, _] : eprop_->rotors) {
      const auto erotor = eprop_->getRotor(link_name);

      if (erotor->channel >= kMaxDshotChannels) {
        qt::qWarnBox(this, "Rotor channel " + QString::number(erotor->channel) + " is not supported.");
        continue;
      }

      registered_.at(erotor->channel) = true;

      const auto text = "CH" + QString::number(erotor->channel) + ": " + QString::fromStdString(link_name);
      rotor_widgets_.at(erotor->channel)->setText(text);

      const auto max_rpm = st::rps2rpm(drone_.prop->maxSpeed(link_name));
      rotor_widgets_.at(erotor->channel)->setMaximumRPM(max_rpm);
    }
  }
  else {
    eprop_.reset();
  }
}

void RotorTestWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  if (eprop_) {
    tar_speeds_pub_ = ros2::createPublisher<tobas_msgs::msg::RotorSpeedArray>(
      node, path::join(ns, kRemoteIfaceNS, topic::kRotorSpeedsCmd));
    dparam_cli_ = std::make_shared<dparam::DynamicParamClient>(node, node::kRpmControlConfigServer, ns);
    get_params_sc_ = std::make_shared<ros2::SyncServiceClient<tobas_dparam_msgs::srv::GetParams>>(
      node, path::join(ns, kRemoteIfaceNS, node::kRpmControlConfigServer, service::kGetDynamicParams));
  }

  node_ = std::move(node);
}

void RotorTestWidget::clearRosInterfaces()
{
  get_params_sc_.reset();
  dparam_cli_.reset();
  tar_speeds_pub_.reset();
  node_.reset();
}

int RotorTestWidget::numRegisteredChannels() const
{
  return st::count(registered_, true);
}

void RotorTestWidget::publishTargetSppeds()
{
  if (!tar_speeds_pub_) {
    return;
  }

  auto tar_speeds = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
  tar_speeds->header.stamp = node_->now();

  for (const auto& [link_name, _] : eprop_->rotors) {
    const auto erotor = eprop_->getRotor(link_name);

    tar_speeds->speeds.emplace_back();
    tar_speeds->speeds.back().link_name = link_name;
    tar_speeds->speeds.back().speed = st::rpm2rps(rotor_widgets_.at(erotor->channel)->getTargetRPM());
  }

  tar_speeds_pub_->publish(std::move(tar_speeds));
}

bool RotorTestWidget::loadCurrentGains()
{
  const auto req = std::make_shared<tobas_dparam_msgs::srv::GetParams::Request>();
  const auto res = get_params_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to get the current RPM control gains.");
    return false;
  }

  for (const auto& [ch, param] : std::views::enumerate(res->params.ints)) {
    TOBAS_CHECK(param.name == paramName(ch));
    rotor_widgets_.at(ch)->setGain(param.value);
  }

  return true;
}

void RotorTestWidget::onStartButtonClicked()
{
  qDebug() << "RotorTestWidget::onStartButtonClicked";

  // Confirm that the vehicle is not armed.
  if (!arming_) {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status has not been received yet.");
    return;
  }
  if (arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed because the rotors are already armed.");
    return;
  }

  // Apply the current gain.
  if (!loadCurrentGains()) {
    return;
  }

  // Enable motor widgets.
  for (const auto& [link_name, _] : eprop_->rotors) {
    const auto erotor = eprop_->getRotor(link_name);
    rotor_widgets_.at(erotor->channel)->setEnabled(true);
  }

  // Temporarily subscribe to rotor states.
  rotor_states_conn_ =
    connect(&bridge_, &rqt::RosQtBridge::rotorStatesReceived, this, &self::rotorStatesCb, Qt::QueuedConnection);

  // Publish commands at a fixed interval.
  update_timer_.start(kUpdatePeriod);

  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  save_button_->setEnabled(true);

  running_ = true;

  qt::qInfoBox(this, "Rotor test started.");
}

void RotorTestWidget::onStopButtonClicked()
{
  qDebug() << "RotorTestWidget::onStopButtonClicked";

  reset();

  qt::qInfoBox(this, "Rotor test stopped.");
}

void RotorTestWidget::onSaveButtonClicked()
{
  qDebug() << "RotorTestWidget::onSaveButtonClicked";

  YAML::Node node(YAML::NodeType::Map);
  for (const auto& [link_name, _] : eprop_->rotors) {
    const auto erotor = eprop_->getRotor(link_name);
    node[paramName(erotor->channel)] = rotor_widgets_.at(erotor->channel)->getGain();
  }

  if (!yaml::save(proj_paths_.rpmCtrlDynParamsPath().toStdString(), node)) {
    qt::qErrorBox(this, "Failed to save the RPM control gains to PC.");
    return;
  }

  qt::qInfoBox(
    this,
    "The RPM control gains have been saved to the local project. "
    "Please click \"Write\" button again to flash them to the FC.");
}

void RotorTestWidget::onTargetRPMChanged(int, size_t)
{
  publishTargetSppeds();
}

void RotorTestWidget::onGainChanged(int gain, size_t ch)
{
  if (dparam_cli_->setInt(paramName(ch), gain) != dparam::DynamicParamClient::kNoError) {
    qWarning() << dparam_cli_->errorMessage();
    qt::qErrorBox(this, "Failed to set the RPM control gain of channel " + QString::number(ch) + ".");
  }
}

void RotorTestWidget::onUpdateTimerTimeout()
{
  publishTargetSppeds();
}

void RotorTestWidget::rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& cur_states)
{
  for (const auto& elem : cur_states->states) {
    if (elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
      continue;
    }

    const auto erotor = eprop_->getRotor(elem.link_name);
    rotor_widgets_.at(erotor->channel)->setCurrentRPM(st::rps2rpm(elem.speed));
  }
}

void RotorTestWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // Force the test to stop if the vehicle is armed while the test is running.
  if (running_ && arming->data) {
    reset();
    qt::qWarnBox(this, "Rotor test was terminated because an arming command was issued.");
  }

  arming_ = arming;
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
