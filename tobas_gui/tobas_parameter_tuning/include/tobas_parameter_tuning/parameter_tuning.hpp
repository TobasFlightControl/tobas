// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <QPushButton>

#include <tobas_drone_core/drone.hpp>
#include <tobas_gui_common/project_paths.hpp>

#include "./param_block.hpp"

namespace tobas
{
namespace gui
{
namespace param
{
class ParameterTuningWidget : public QWidget
{
  Q_OBJECT

  using self = ParameterTuningWidget;
  using super = QWidget;

  static constexpr size_t kNumBlocks = 4;
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit ParameterTuningWidget();

  void reset();
  void updateProject(const std::filesystem::path& proj_path);
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

private:
  cmn::ProjectPaths proj_paths_;
  Drone drone_;

  const std::array<std::string, kNumBlocks> file_names_;
  const std::array<ParamBlockWidget*, kNumBlocks> blocks_;

  QPushButton* load_button_;
  QPushButton* save_button_;
  QPushButton* dflt_button_;

  bool project_loaded_ = false;
  bool ros_initialized_ = false;

  void updateOperationButtons();

private Q_SLOTS:
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onDefaultButtonClicked();
};
}  // namespace param
}  // namespace gui
}  // namespace tobas
