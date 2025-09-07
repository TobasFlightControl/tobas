#include "tobas_parameter_tuning/parameter_tuning.hpp"

#include <ranges>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace param
{
ParameterTuningWidget::ParameterTuningWidget(rclcpp::Node::SharedPtr node)
  : file_names_{ common::ProjectPaths::kImuFilterDynamicParamFileName,
                 common::ProjectPaths::kObserverDynamicParamFileName,
                 common::ProjectPaths::kControllerDynamicParamFileName,
                 common::ProjectPaths::kRcTeleopDynamicParamFileName }
  , blocks_{ new ParamBlockWidget(node, tobas::node::kImuFilterConfigServer, "IMU Filter"),
             new ParamBlockWidget(node, tobas::node::kObserver, "State Estimator"),
             new ParamBlockWidget(node, tobas::node::kController, "Flight Controller"),
             new ParamBlockWidget(node, tobas::node::kRcTeleop, "Radio Control") }
{
  load_button_ = new QPushButton("Load");
  save_button_ = new QPushButton("Save");
  reset_button_ = new QPushButton("Reset");

  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  reset_button_->setFixedSize(kButtonWidth, kButtonHeight);

  reset();
  load_button_->setEnabled(false);  // プロジェクトが読み込まれるまではLoadボタンを押せないように

  // Layout
  const auto root_rows = new QVBoxLayout();
  setLayout(root_rows);

  const auto button_cols = new QHBoxLayout();
  root_rows->addLayout(button_cols);
  button_cols->addWidget(load_button_);
  button_cols->addWidget(save_button_);
  button_cols->addWidget(reset_button_);
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
}

void ParameterTuningWidget::reset()
{
  load_button_->setEnabled(true);
  save_button_->setEnabled(false);
  reset_button_->setEnabled(false);

  for (const auto& block : blocks_) {
    block->clear();
    block->setVisible(false);
  }
}

bool ParameterTuningWidget::updateProject(const fs::path& proj_path)
{
  reset();

  // Update project path
  proj_paths_.setProjPath(proj_path);

  // Load drone configuration
  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  if (!drone_.load(tbsdrn_path)) {
    qt::qErrorBox(this, "Failed to load drone configuration.");
    return false;
  }

  return true;
}

void ParameterTuningWidget::onLoadButtonClicked()
{
  for (const auto& block : blocks_) {
    if (!block->load(drone_.name)) {
      return;
    }
  }

  // 読み込みと同時に可視化
  for (const auto& block : blocks_) {
    block->setVisible(true);
  }

  save_button_->setEnabled(true);
  reset_button_->setEnabled(true);

  qt::qInfoBox(this, "Dynamic parameters are loaded successfully.");
}

void ParameterTuningWidget::onSaveButtonClicked()
{
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

void ParameterTuningWidget::onResetButtonClicked()
{
  // 本当に全てのパラメータをリセットしてよいか確認
  if (!qt::yesOrNo(this, "Are you sure you want to reset all parameters to their defaults?", qt::QMessageLevel::WARN)) {
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
