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
ParameterTuningWidget::ParameterTuningWidget()
  : file_names_{ "imu_filter_dynamic.yaml", "observer_dynamic.yaml", "controller_dynamic.yaml", "rc_teleop_dynamic.yaml" }
  , blocks_{ new ParamBlockWidget(node::kImuFilterConfigServer, "IMU Filter"),
             new ParamBlockWidget(node::kObserver, "State Estimator"),
             new ParamBlockWidget(node::kController, "Flight Controller"),
             new ParamBlockWidget(node::kRcTeleop, "Radio Control") }
{
  load_button_ = new QPushButton("Load");
  load_button_->setToolTip("Load current dynamic parameters from the connected vehicle.");

  save_button_ = new QPushButton("Save");
  save_button_->setToolTip("Save current dynamic parameters to the local project.");

  reset_button_ = new QPushButton("Reset");
  reset_button_->setToolTip("Reset all dynamic parameters to their initial values.");

  default_button_ = new QPushButton("Default");
  default_button_->setToolTip("Restore all dynamic parameters to their default values.");

  constexpr int kButtonWidth = 100;
  constexpr int kButtonHeight = 40;
  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  reset_button_->setFixedSize(kButtonWidth, kButtonHeight);
  default_button_->setFixedSize(kButtonWidth, kButtonHeight);

  reset();

  // Layout
  const auto root_rows = new QVBoxLayout();
  setLayout(root_rows);

  const auto button_cols = new QHBoxLayout();
  root_rows->addLayout(button_cols);
  button_cols->addWidget(load_button_);
  button_cols->addWidget(save_button_);
  button_cols->addSpacing(50);
  button_cols->addWidget(reset_button_);
  button_cols->addWidget(default_button_);
  button_cols->addStretch();

  const auto param_rows = qt::createScrollableQVBoxLayout(root_rows);
  for (const auto& block : blocks_) {
    param_rows->addWidget(block);
  }
  param_rows->addStretch();

  // Connection
  connect(load_button_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(save_button_, &QPushButton::clicked, this, &self::onSaveButtonClicked);
  connect(reset_button_, &QPushButton::clicked, this, &self::onResetButtonClicked);
  connect(default_button_, &QPushButton::clicked, this, &self::onDefaultButtonClicked);
}

void ParameterTuningWidget::reset()
{
  load_button_->setEnabled(project_loaded_ && ros_initialized_);
  save_button_->setEnabled(false);
  reset_button_->setEnabled(false);
  default_button_->setEnabled(false);

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
  reset_button_->setEnabled(true);
  default_button_->setEnabled(true);

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
    "Please click 'Write' button again to flash them to the FC.");
}

void ParameterTuningWidget::onResetButtonClicked()
{
  qDebug() << "ParameterTuningWidget::onResetButtonClicked";

  if (!qt::yesOrNo(this, "Are you sure you want to reset all parameters to their initial values?", qt::WARN)) {
    return;
  }

  for (const auto& block : blocks_) {
    if (!block->setToInitialValues()) {
      return;
    }
  }

  qt::qInfoBox(this, "Dynamic parameters have been set to their initial values successfully.");
}

void ParameterTuningWidget::onDefaultButtonClicked()
{
  qDebug() << "ParameterTuningWidget::onDefaultButtonClicked";

  if (!qt::yesOrNo(this, "Are you sure you want to reset all parameters to their default values?", qt::WARN)) {
    return;
  }

  for (const auto& block : blocks_) {
    if (!block->setToDefaultValues()) {
      return;
    }
  }

  qt::qInfoBox(this, "Dynamic parameters have been set to their default values successfully.");
}
}  // namespace param
}  // namespace gui
}  // namespace tobas
