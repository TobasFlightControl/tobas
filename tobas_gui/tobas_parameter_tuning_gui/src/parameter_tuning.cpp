#include "tobas_parameter_tuning_gui/parameter_tuning.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_gui_common/path.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace param
{
ParameterTuningWidget::ParameterTuningWidget(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
  load_button_ = new QPushButton("Load");
  save_button_ = new QPushButton("Save");
  reset_button_ = new QPushButton("Reset");

  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  reset_button_->setFixedSize(kButtonWidth, kButtonHeight);

  imu_filter_params_ = new ParamBlockWidget(node, "IMU Filter");
  observer_params_ = new ParamBlockWidget(node, "State Estimator");
  controller_params_ = new ParamBlockWidget(node, "Flight Controller");
  rc_teleop_params_ = new ParamBlockWidget(node, "Radio Control");

  reset();

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
  param_rows->addWidget(imu_filter_params_);
  param_rows->addWidget(observer_params_);
  param_rows->addWidget(controller_params_);
  param_rows->addWidget(rc_teleop_params_);
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

  imu_filter_params_->clear();
  observer_params_->clear();
  controller_params_->clear();
  rc_teleop_params_->clear();

  imu_filter_params_->setVisible(false);
  observer_params_->setVisible(false);
  controller_params_->setVisible(false);
  rc_teleop_params_->setVisible(false);
}

bool ParameterTuningWidget::updateTBSPath(const fs::path& tbs_path)
{
  reset();

  if (!drone_.load(common::getProjTbsDrnPath(tbs_path))) {
    qt::qErrorBox(this, "Failed to load drone configuration.");
    return false;
  }

  tbs_path_ = tbs_path;

  return true;
}

bool ParameterTuningWidget::saveLocal()
{
  if (!imu_filter_params_->saveLocal(common::getProjImuFiltDynParamsPath(tbs_path_))) {
    return false;
  }
  if (!observer_params_->saveLocal(common::getProjObsvDynParamsPath(tbs_path_))) {
    return false;
  }
  if (!controller_params_->saveLocal(common::getProjCtrlDynParamsPath(tbs_path_))) {
    return false;
  }
  if (!rc_teleop_params_->saveLocal(common::getProjRcTeleopDynParamsPath(tbs_path_))) {
    return false;
  }

  return true;
}

bool ParameterTuningWidget::saveRemote()
{
  const auto remote_tbs_path = common::getProjRemotePath(tbs_path_);

  if (!imu_filter_params_->saveRemote(common::getProjImuFiltDynParamsPath(remote_tbs_path))) {
    return false;
  }
  if (!observer_params_->saveRemote(common::getProjObsvDynParamsPath(remote_tbs_path))) {
    return false;
  }
  if (!controller_params_->saveRemote(common::getProjCtrlDynParamsPath(remote_tbs_path))) {
    return false;
  }
  if (!rc_teleop_params_->saveRemote(common::getProjRcTeleopDynParamsPath(remote_tbs_path))) {
    return false;
  }

  return true;
}

void ParameterTuningWidget::onLoadButtonClicked()
{
  if (drone_.name.empty()) {
    qt::qWarnBox(this, "Tobas project is not loaded yet.");
    return;
  }

  if (!imu_filter_params_->load(drone_.name, tobas::node::kImuFilterConfigServer)) {
    return;
  }
  if (!observer_params_->load(drone_.name, tobas::node::kObserver)) {
    return;
  }
  if (!controller_params_->load(drone_.name, tobas::node::kController)) {
    return;
  }
  if (!rc_teleop_params_->load(drone_.name, tobas::node::kRcTeleop)) {
    return;
  }

  // 読み込みと同時に可視化
  imu_filter_params_->setVisible(true);
  observer_params_->setVisible(true);
  controller_params_->setVisible(true);
  rc_teleop_params_->setVisible(true);

  save_button_->setEnabled(true);
  reset_button_->setEnabled(true);

  qt::qInfoBox(this, "Dynamic parameters are loaded successfully.");
}

void ParameterTuningWidget::onSaveButtonClicked()
{
  if (ssh_client_.connect() == ssh::SSHClient::kNoError) {
    if (!saveRemote()) {
      return;
    }
    if (!saveLocal()) {
      return;
    }

    qt::qInfoBox(this, "Dynamic parameters are saved to PC and FC successfully.");
  }
  else {
    if (!qt::yesOrNo(
          this, "Failed to connect to FC. Do you want to save parameters only to PC?", qt::QMessageLevel::WARN)) {
      return;
    }

    if (!saveLocal()) {
      return;
    }

    qt::qInfoBox(this, "Dynamic parameters are saved only to PC. Write the Tobas project to apply them to the FC.");
  }
}

void ParameterTuningWidget::onResetButtonClicked()
{
  // 本当に全てのパラメータをリセットしてよいか確認
  if (!qt::yesOrNo(this, "Are you sure you want to reset all parameters to their defaults?", qt::QMessageLevel::WARN)) {
    return;
  }

  if (!imu_filter_params_->setToDefaults()) {
    return;
  }
  if (!observer_params_->setToDefaults()) {
    return;
  }
  if (!controller_params_->setToDefaults()) {
    return;
  }
  if (!rc_teleop_params_->setToDefaults()) {
    return;
  }

  qt::qInfoBox(this, "Dynamic parameters are set to their defaults successfully.");
}
}  // namespace param
}  // namespace gui
