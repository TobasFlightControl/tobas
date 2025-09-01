#pragma once

#include <filesystem>

#include <QPushButton>

#include <tobas_drone_core/drone.hpp>
#include <tobas_gui_common/project_paths.hpp>

#include "./param_block.hpp"

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
  explicit ParameterTuningWidget(rclcpp::Node::SharedPtr node);

  void reset();
  bool updateProject(const std::filesystem::path& proj_path);

private:
  common::ProjectPaths proj_paths_;
  tobas::Drone drone_;

  const std::array<std::string, kNumBlocks> file_names_;
  const std::array<ParamBlockWidget*, kNumBlocks> blocks_;

  QPushButton* load_button_;
  QPushButton* save_button_;
  QPushButton* reset_button_;

private Q_SLOTS:
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onResetButtonClicked();
};
}  // namespace param
}  // namespace gui
