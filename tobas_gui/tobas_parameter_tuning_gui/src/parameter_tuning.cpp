#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_gui_common/package.hpp>

#include "tobas_parameter_tuning_gui/parameter_tuning.hpp"

namespace gui
{
namespace param_tuning
{
ParameterTuningWidget::ParameterTuningWidget(rclcpp::Node::SharedPtr node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  load_button_ = new QPushButton("Load");
  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(load_button_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  cols->addWidget(load_button_);

  save_button_ = new QPushButton("Save");
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(save_button_, &QPushButton::clicked, this, &self::onSaveButtonClicked);
  cols->addWidget(save_button_);

  reset_button_ = new QPushButton("Reset");
  reset_button_->setFixedSize(kButtonWidth, kButtonHeight);
  connect(reset_button_, &QPushButton::clicked, this, &self::onResetButtonClicked);
  cols->addWidget(reset_button_);

  cols->addStretch();

  const auto param_rows = qt::createScrollableQVBoxLayout(rows);

  controller_params_ = new ParamBlockWidget(node, "Controller");
  param_rows->addWidget(controller_params_);

  observer_params_ = new ParamBlockWidget(node, "Observer");
  param_rows->addWidget(observer_params_);

  param_rows->addStretch();

  reset();
}

void ParameterTuningWidget::reset()
{
  tbs_path_.clear();
  drone_ = tobas::Drone();

  load_button_->setEnabled(false);
  save_button_->setEnabled(false);
  reset_button_->setEnabled(false);

  controller_params_->clear();
  observer_params_->clear();

  controller_params_->setVisible(false);
  observer_params_->setVisible(false);
}

bool ParameterTuningWidget::updateTBSPath(const std::filesystem::path& tbs_path)
{
  reset();

  if (!drone_.load(common::getTBSDRNPath(tbs_path)))
  {
    qt::qErrorBox(this, "Failed to load drone configurations.");
    return false;
  }

  tbs_path_ = tbs_path;

  load_button_->setEnabled(true);
  save_button_->setEnabled(false);
  reset_button_->setEnabled(false);

  return true;
}

void ParameterTuningWidget::onLoadButtonClicked()
{
  if (!controller_params_->load(drone_.name, tobas::kControllerNode))
    return;

  if (!observer_params_->load(drone_.name, tobas::kObserverNode))
    return;

  // 読み込みと同時に可視化
  controller_params_->setVisible(true);
  observer_params_->setVisible(true);

  save_button_->setEnabled(true);
  reset_button_->setEnabled(true);

  qt::qInfoBox(this, "Dynamic parameters are loaded successfully.");
}

void ParameterTuningWidget::onSaveButtonClicked()
{
  const auto remote_tbs_path = common::getRemoteTBSPath(tbs_path_);

  const auto ctrl_path_local = common::getControllerDynamicParamsPath(tbs_path_);
  const auto ctrl_path_remote = common::getControllerDynamicParamsPath(remote_tbs_path);
  if (!controller_params_->save(ctrl_path_local, ctrl_path_remote))
    return;

  const auto obsv_path_local = common::getObserverDynamicParamsPath(tbs_path_);
  const auto obsv_path_remote = common::getObserverDynamicParamsPath(remote_tbs_path);
  if (!observer_params_->save(obsv_path_local, obsv_path_remote))
    return;

  qt::qInfoBox(this, "Dynamic parameters are saved to PC and FC successfully.");
}

void ParameterTuningWidget::onResetButtonClicked()
{
  // 本当に全てのパラメータをリセットしてよいか確認
  if (!qt::yesOrNo(this, "Are you sure you want to reset all parameters to their defaults?", qt::QMessageLevel::WARN))
    return;

  if (!controller_params_->setToDefaults())
    return;

  if (!observer_params_->setToDefaults())
    return;

  qt::qInfoBox(this, "Dynamic parameters are set to their defaults successfully.");
}
}  // namespace param_tuning
}  // namespace gui
