// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_parameter_tuning/param_block.hpp"

#include <QDebug>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QStyle>
#include <QVBoxLayout>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_yaml_tools/core.hpp>

using namespace std::placeholders;

namespace tobas
{
namespace gui
{
namespace param
{
namespace
{
template <typename Config>
void updateButtonStates(const Config& _config)
{
  const auto value = _config.slider->value();
  _config.down_button_->setEnabled(value > _config.slider->minimum());
  _config.up_button_->setEnabled(value < _config.slider->maximum());
  _config.reset_button_->setEnabled(value != _config.initial_value);
  _config.home_button_->setEnabled(value != _config.default_value);
}
}  // namespace

ParamBlockWidget::ParamBlockWidget(const std::string& _node_name, const QString& _label) : node_name_(_node_name)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  label_ = new QLabel(_label);
  label_->setFont(qt::DefaultFont(12, QFont::Bold));
  qt::addWidgetCenter(label_, rows);

  form_ = new qt::FormLayout();
  rows->addLayout(form_);
}

void ParamBlockWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns)
{
  const auto get_param_srv = path::join(_ns, kRemoteIfaceNS, node_name_, service::kGetDynamicParams);
  get_param_sc_.emplace(_node, get_param_srv);

  dparam_cli_.emplace(_node, node_name_, _ns);
}

void ParamBlockWidget::clearRosInterfaces()
{
  dparam_cli_.reset();
  get_param_sc_.reset();
}

bool ParamBlockWidget::load()
{
  constexpr int kParamNameWidth = 250;
  constexpr int kLineEditWidth = 150;

  clear();

  if (!get_param_sc_) {
    qt::qErrorBox(this, "ROS interfaces have not been initialized.");
    return false;
  }

  // Get dynamic parameters.
  const auto req = std::make_shared<tobas_dparam_msgs::srv::GetParams::Request>();
  const auto res = get_param_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to get dynamic parameters configuration of '" + label_->text() + "'.");
    return false;
  }
  const auto& params = res->params;

  // Add sliders.
  constexpr char kDecreaseButtonTooltip[] = "Decrease by one step";
  constexpr char kIncreaseButtonTooltip[] = "Increase by one step";
  constexpr char kResetButtonTooltip[] = "Reset to initial value";
  constexpr char kHomeButtonTooltip[] = "Restore default value";

  const auto down_icon = style()->standardIcon(QStyle::SP_ArrowDown);
  const auto up_icon = style()->standardIcon(QStyle::SP_ArrowUp);
  const auto reset_icon = style()->standardIcon(QStyle::SP_BrowserReload);
  const auto home_icon = QIcon::fromTheme("go-home");

  for (const auto& param : params.ints) {
    const auto param_name_label = new QLabel(QString::fromStdString(param.name));
    param_name_label->setFixedWidth(kParamNameWidth);

    IntConfig config;
    config.step = param.step;
    config.default_value = param.default_value;
    config.initial_value = param.initial_value;
    config.prefix = QString::fromStdString(str::convertToSuperscript(param.prefix));

    config.down_button_ = new QPushButton();
    config.down_button_->setIcon(down_icon);
    config.down_button_->setToolTip(kDecreaseButtonTooltip);

    config.up_button_ = new QPushButton();
    config.up_button_->setIcon(up_icon);
    config.up_button_->setToolTip(kIncreaseButtonTooltip);

    config.reset_button_ = new QPushButton();
    config.reset_button_->setIcon(reset_icon);
    config.reset_button_->setToolTip(kResetButtonTooltip);

    config.home_button_ = new QPushButton();
    config.home_button_->setIcon(home_icon);
    config.home_button_->setToolTip(kHomeButtonTooltip);

    config.slider = new qt::Slider(Qt::Horizontal);
    config.slider->setRange(param.minimum_value, param.maximum_value);
    config.slider->setValue(param.current_value);

    config.line_edit = new QLineEdit();
    config.line_edit->setFixedWidth(kLineEditWidth);
    config.line_edit->setAlignment(Qt::AlignRight);
    config.line_edit->setReadOnly(true);
    config.line_edit->setText(QString::number(param.step * param.current_value) + config.prefix);

    updateButtonStates(config);
    int_configs_[param.name] = config;

    const auto cols = new QHBoxLayout();
    cols->addWidget(config.down_button_);
    cols->addWidget(config.up_button_);
    cols->addWidget(config.reset_button_);
    cols->addWidget(config.home_button_);
    cols->addWidget(config.slider);
    cols->addWidget(config.line_edit);
    form_->addRow(param_name_label, cols);

    connect(config.down_button_, &QPushButton::clicked, std::bind(&self::onIntDownButtonClicked, this, param.name));
    connect(config.up_button_, &QPushButton::clicked, std::bind(&self::onIntUpButtonClicked, this, param.name));
    connect(config.reset_button_, &QPushButton::clicked, std::bind(&self::onIntResetButtonClicked, this, param.name));
    connect(config.home_button_, &QPushButton::clicked, std::bind(&self::onIntHomeButtonClicked, this, param.name));
    connect(config.slider, &qt::Slider::valueChanged, std::bind(&self::onIntSliderValueChanged, this, _1, param.name));
  }

  for (const auto& param : params.doubles) {
    const auto param_name_label = new QLabel(QString::fromStdString(param.name));
    param_name_label->setFixedWidth(kParamNameWidth);

    DoubleConfig config;
    config.step = param.step;
    config.default_value = param.default_value;
    config.initial_value = param.initial_value;
    config.prefix = QString::fromStdString(str::convertToSuperscript(param.prefix));

    config.down_button_ = new QPushButton();
    config.down_button_->setIcon(down_icon);
    config.down_button_->setToolTip(kDecreaseButtonTooltip);

    config.up_button_ = new QPushButton();
    config.up_button_->setIcon(up_icon);
    config.up_button_->setToolTip(kIncreaseButtonTooltip);

    config.reset_button_ = new QPushButton();
    config.reset_button_->setIcon(reset_icon);
    config.reset_button_->setToolTip(kResetButtonTooltip);

    config.home_button_ = new QPushButton();
    config.home_button_->setIcon(home_icon);
    config.home_button_->setToolTip(kHomeButtonTooltip);

    config.slider = new qt::Slider(Qt::Horizontal);
    config.slider->setRange(param.minimum_value, param.maximum_value);
    config.slider->setValue(param.current_value);

    config.line_edit = new QLineEdit();
    config.line_edit->setFixedWidth(kLineEditWidth);
    config.line_edit->setAlignment(Qt::AlignRight);
    config.line_edit->setReadOnly(true);
    config.line_edit->setText(QString::number(param.step * param.current_value) + config.prefix);

    updateButtonStates(config);
    double_configs_[param.name] = config;

    const auto cols = new QHBoxLayout();
    cols->addWidget(config.down_button_);
    cols->addWidget(config.up_button_);
    cols->addWidget(config.reset_button_);
    cols->addWidget(config.home_button_);
    cols->addWidget(config.slider);
    cols->addWidget(config.line_edit);
    form_->addRow(param_name_label, cols);

    connect(config.down_button_, &QPushButton::clicked, std::bind(&self::onDoubleDownButtonClicked, this, param.name));
    connect(config.up_button_, &QPushButton::clicked, std::bind(&self::onDoubleUpButtonClicked, this, param.name));
    connect(config.reset_button_, &QPushButton::clicked, std::bind(&self::onDoubleResetButtonClicked, this, param.name));
    connect(config.home_button_, &QPushButton::clicked, std::bind(&self::onDoubleHomeButtonClicked, this, param.name));
    connect(
      config.slider, &qt::Slider::valueChanged, std::bind(&self::onDoubleSliderValueChanged, this, _1, param.name));
  }

  return true;
}

bool ParamBlockWidget::save(const QString& _path)
{
  const auto config = createCurrentConfig();

  // Confirm that the configuration file exists.
  if (!QFileInfo(_path).isFile()) {
    qt::qErrorBox(this, _path + " does not exist on PC.");
    return false;
  }

  // Save to the PC.
  if (!yaml::save(_path.toStdString(), config)) {
    qt::qErrorBox(this, "Failed to save configuration to PC.");
    return false;
  }

  return true;
}

void ParamBlockWidget::clear()
{
  form_->clear();
  int_configs_.clear();
  double_configs_.clear();
}

bool ParamBlockWidget::setToInitialValues()
{
  if (!dparam_cli_) {
    qt::qErrorBox(this, "ROS interfaces have not been initialized.");
    return false;
  }

  for (const auto& [name, config] : int_configs_) {
    if (!setValue(name, config, config.initial_value)) {
      return false;
    }
  }

  for (const auto& [name, config] : double_configs_) {
    if (!setValue(name, config, config.initial_value)) {
      return false;
    }
  }

  return true;
}

bool ParamBlockWidget::setToDefaultValues()
{
  if (!dparam_cli_) {
    qt::qErrorBox(this, "ROS interfaces have not been initialized.");
    return false;
  }

  for (const auto& [name, config] : int_configs_) {
    if (!setValue(name, config, config.default_value)) {
      return false;
    }
  }

  for (const auto& [name, config] : double_configs_) {
    if (!setValue(name, config, config.default_value)) {
      return false;
    }
  }

  return true;
}

YAML::Node ParamBlockWidget::createCurrentConfig() const
{
  YAML::Node res(YAML::NodeType::Map);

  for (const auto& [name, config] : int_configs_) {
    res[name] = config.slider->value();
  }

  for (const auto& [name, config] : double_configs_) {
    res[name] = config.slider->value();
  }

  return res;
}

template <typename Config>
bool ParamBlockWidget::setValue(const std::string& _name, const Config& _config, int32_t _value)
{
  if (_config.slider->value() == _value) {
    return true;
  }

  if (dparam_cli_->setInt(_name, _value) != dparam::DynamicParamClient::kNoError) {
    qWarning() << dparam_cli_->errorMessage();
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter '" + _name.c_str() + "'.");
    return false;
  }

  const QSignalBlocker block(_config.slider);
  _config.slider->setValue(_value);
  _config.line_edit->setText(QString::number(_config.step * _value) + _config.prefix);
  updateButtonStates(_config);

  return true;
}

void ParamBlockWidget::onIntDownButtonClicked(const std::string& _name)
{
  auto& config = int_configs_.at(_name);
  config.slider->setValue(config.slider->value() - 1);
}

void ParamBlockWidget::onIntUpButtonClicked(const std::string& _name)
{
  auto& config = int_configs_.at(_name);
  config.slider->setValue(config.slider->value() + 1);
}

void ParamBlockWidget::onIntResetButtonClicked(const std::string& _name)
{
  auto& config = int_configs_.at(_name);
  config.slider->setValue(config.initial_value);
}

void ParamBlockWidget::onIntHomeButtonClicked(const std::string& _name)
{
  auto& config = int_configs_.at(_name);
  config.slider->setValue(config.default_value);
}

void ParamBlockWidget::onIntSliderValueChanged(long _value, const std::string& _name)
{
  auto& config = int_configs_.at(_name);
  config.line_edit->setText(QString::number(config.step * _value) + config.prefix);
  updateButtonStates(config);

  if (dparam_cli_->setInt(_name, _value) != dparam::DynamicParamClient::kNoError) {
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter '" + _name.c_str() + "'.");
  }
}

void ParamBlockWidget::onDoubleDownButtonClicked(const std::string& _name)
{
  auto& config = double_configs_.at(_name);
  config.slider->setValue(config.slider->value() - 1);
}

void ParamBlockWidget::onDoubleUpButtonClicked(const std::string& _name)
{
  auto& config = double_configs_.at(_name);
  config.slider->setValue(config.slider->value() + 1);
}

void ParamBlockWidget::onDoubleResetButtonClicked(const std::string& _name)
{
  if (!qt::yesOrNo(
        this,
        "Are you sure you want to reset parameter '" + QString::fromStdString(_name) + "' to its initial value?",
        qt::WARN)) {
    return;
  }

  auto& config = double_configs_.at(_name);
  config.slider->setValue(config.initial_value);
}

void ParamBlockWidget::onDoubleHomeButtonClicked(const std::string& _name)
{
  if (!qt::yesOrNo(
        this,
        "Are you sure you want to reset parameter '" + QString::fromStdString(_name) + "' to its default value?",
        qt::WARN)) {
    return;
  }

  auto& config = double_configs_.at(_name);
  config.slider->setValue(config.default_value);
}

void ParamBlockWidget::onDoubleSliderValueChanged(long _value, const std::string& _name)
{
  auto& config = double_configs_.at(_name);
  config.line_edit->setText(QString::number(config.step * _value) + config.prefix);
  updateButtonStates(config);

  if (dparam_cli_->setDouble(_name, _value) != dparam::DynamicParamClient::kNoError) {
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter '" + _name.c_str() + "'.");
  }
}
}  // namespace param
}  // namespace gui
}  // namespace tobas
