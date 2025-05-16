#pragma once

#include <filesystem>

#include <QPushButton>

#include <tobas_drone_core/drone.hpp>

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

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

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
  ParamBlockWidget* rc_teleop_params_;
  ParamBlockWidget* imu_preprocess_params_;

private Q_SLOTS:
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onResetButtonClicked();
};
}  // namespace param
}  // namespace gui
