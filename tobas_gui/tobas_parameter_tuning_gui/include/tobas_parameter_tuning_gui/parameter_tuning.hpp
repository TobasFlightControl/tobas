#pragma once

#include <filesystem>
#include <QPushButton>

#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "./param_block.hpp"

namespace gui
{
namespace param_tuning
{
class ParameterTuningWidget : public QWidget
{
  Q_OBJECT

  using self = ParameterTuningWidget;
  using super = QWidget;

  static constexpr int kButtonHeight = 100;
  static constexpr int kButtonWidth = 40;

public:
  explicit ParameterTuningWidget(rclcpp::Node::SharedPtr node);

  void reset();
  bool updateTBSPath(const std::filesystem::path& tbs_path);

private:
  std::filesystem::path tbs_path_;
  tobas::Drone drone_;

  QPushButton* load_button_;
  QPushButton* save_button_;
  QPushButton* reset_button_;

  ParamBlockWidget* controller_params_;
  ParamBlockWidget* observer_params_;

private Q_SLOTS:
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onResetButtonClicked();
};
}  // namespace param_tuning
}  // namespace gui
