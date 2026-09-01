// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_parameter_tuning/parameter_tuning.hpp"

#include <ranges>

#include <QDebug>
#include <QDir>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/node.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace gui
{
namespace param
{
namespace
{
constexpr char kImuFilterDynamicParamFileName[] = "imu_filter_dynamic.yaml";
constexpr char kObserverDynamicParamFileName[] = "observer_dynamic.yaml";
constexpr char kControllerDynamicParamFileName[] = "controller_dynamic.yaml";
constexpr char kRcTeleopDynamicParamFileName[] = "rc_teleop_dynamic.yaml";

constexpr int kButtonWidth = 100;
constexpr int kButtonHeight = 40;
}  // namespace

ParameterTuningWidget::ParameterTuningWidget()
  : file_names_{ kImuFilterDynamicParamFileName,
                 kObserverDynamicParamFileName,
                 kControllerDynamicParamFileName,
                 kRcTeleopDynamicParamFileName }
  , blocks_{ new ParamBlockWidget(node::kImuFilterConfigServer, "IMU Filter"),
             new ParamBlockWidget(node::kObserver, "State Estimator"),
             new ParamBlockWidget(node::kController, "Flight Controller"),
             new ParamBlockWidget(node::kRcTeleop, "Radio Control") }
{
  load_button_ = new QPushButton("Load");
  save_button_ = new QPushButton("Save");
  dflt_button_ = new QPushButton("Default");

  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  dflt_button_->setFixedSize(kButtonWidth, kButtonHeight);

  reset();

  // Layout
  const auto root_rows = new QVBoxLayout();
  setLayout(root_rows);

  const auto button_cols = new QHBoxLayout();
  root_rows->addLayout(button_cols);
  button_cols->addWidget(load_button_);
  button_cols->addWidget(save_button_);
  button_cols->addWidget(dflt_button_);
  button_cols->addStretch();

  const auto param_rows = qt::createScrollableQVBoxLayout(root_rows);
  for (const auto& block : blocks_) {
    param_rows->addWidget(block);
  }
  param_rows->addStretch();

  // Connection
  connect(load_button_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(save_button_, &QPushButton::clicked, this, &self::onSaveButtonClicked);
  connect(dflt_button_, &QPushButton::clicked, this, &self::onDefaultButtonClicked);
}

void ParameterTuningWidget::reset()
{
  load_button_->setEnabled(project_loaded_ && ros_initialized_);
  save_button_->setEnabled(false);
  dflt_button_->setEnabled(false);

  for (const auto& block : blocks_) {
    block->clear();
    block->setVisible(false);
  }
}

void ParameterTuningWidget::updateProject(const QString& proj_path)
{
  // Update project path.
  proj_paths_.setProjPath(proj_path);

  // Load drone configuration.
  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  TOBAS_CHECK(drone_.load(tbsdrn_path.toStdString()));

  project_loaded_ = true;
}

void ParameterTuningWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  for (const auto& block : blocks_) {
    block->initializeRosInterfaces(node, ns);
  }

  ros_initialized_ = true;
}

void ParameterTuningWidget::clearRosInterfaces()
{
  for (const auto& block : blocks_) {
    block->clearRosInterfaces();
  }

  ros_initialized_ = false;
}

void ParameterTuningWidget::onLoadButtonClicked()
{
  qDebug() << "ParameterTuningWidget::onLoadButtonClicked";

  for (const auto& block : blocks_) {
    if (!block->load()) {
      return;
    }
  }

  // Visualize as soon as loading completes.
  for (const auto& block : blocks_) {
    block->setVisible(true);
  }

  save_button_->setEnabled(true);
  dflt_button_->setEnabled(true);

  qt::qInfoBox(this, "Dynamic parameters are loaded successfully.");
}

void ParameterTuningWidget::onSaveButtonClicked()
{
  qDebug() << "ParameterTuningWidget::onSaveButtonClicked";

  const auto config_dir_path = proj_paths_.cfgConfigDirPath();

  for (const auto& [block, file_name] : std::views::zip(blocks_, file_names_)) {
    const auto file_path = QDir(config_dir_path).filePath(file_name);
    if (!block->save(file_path)) {
      return;
    }
  }

  qt::qInfoBox(
    this,
    "Dynamic parameters have been saved to the local project. "
    "Please click \"Write\" button again to flash them to the FC.");
}

void ParameterTuningWidget::onDefaultButtonClicked()
{
  qDebug() << "ParameterTuningWidget::onDefaultButtonClicked";

  // Confirm before resetting all parameters.
  if (!qt::yesOrNo(this, "Are you sure you want to reset all parameters to their defaults?", qt::WARN)) {
    return;
  }

  for (const auto& block : blocks_) {
    if (!block->setToDefaults()) {
      return;
    }
  }

  qt::qInfoBox(this, "Dynamic parameters are set to their defaults successfully.");
}
}  // namespace param
}  // namespace gui
}  // namespace tobas
