// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_parameter_tuning/parameter_tuning.hpp"

#include <ranges>

#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/node.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace param
{
ParameterTuningWidget::ParameterTuningWidget(rclcpp::Node::SharedPtr node)
  : file_names_{ cmn::ProjectPaths::kImuFilterDynamicParamFileName,
                 cmn::ProjectPaths::kObserverDynamicParamFileName,
                 cmn::ProjectPaths::kControllerDynamicParamFileName,
                 cmn::ProjectPaths::kRcTeleopDynamicParamFileName }
  , blocks_{ new ParamBlockWidget(node, node::kImuFilterConfigServer, "IMU Filter"),
             new ParamBlockWidget(node, node::kObserver, "State Estimator"),
             new ParamBlockWidget(node, node::kController, "Flight Controller"),
             new ParamBlockWidget(node, node::kRcTeleop, "Radio Control") }
{
  load_button_ = new QPushButton("Load");
  save_button_ = new QPushButton("Save");
  dflt_button_ = new QPushButton("Default");

  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  dflt_button_->setFixedSize(kButtonWidth, kButtonHeight);

  reset();
  load_button_->setEnabled(false);  // Disable the Load button until a project has been loaded.

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
  load_button_->setEnabled(true);
  save_button_->setEnabled(false);
  dflt_button_->setEnabled(false);

  for (const auto& block : blocks_) {
    block->clear();
    block->setVisible(false);
  }
}

bool ParameterTuningWidget::updateProject(const fs::path& proj_path)
{
  reset();

  // Update project path.
  proj_paths_.setProjPath(proj_path);

  // Load drone configuration.
  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  if (!drone_.load(tbsdrn_path)) {
    qt::qErrorBox(this, "Failed to load drone configuration.");
    return false;
  }

  // Set namespace.
  const auto ns = '/' + drone_.name;
  for (const auto& block : blocks_) {
    block->setNamespace(ns);
  }

  return true;
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
    const auto file_path = config_dir_path / file_name;
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
