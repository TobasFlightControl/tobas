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

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace tobas
{
namespace gui
{
namespace param
{
namespace
{
constexpr int kLabelPSize = 12;
constexpr int kParamNameWidth = 250;
constexpr int kLineEditWidth = 150;
}  // namespace

ParamBlockWidget::ParamBlockWidget(const std::string& node_name, const QString& label) : node_name_(node_name)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  label_ = new QLabel(label);
  label_->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  qt::addWidgetCenter(label_, rows);

  form_ = new qt::FormLayout();
  rows->addLayout(form_);
}

void ParamBlockWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  const auto get_param_srv = path::join(ns, kRemoteIfaceNS, node_name_, service::kGetDynamicParams);
  get_param_sc_.emplace(node, get_param_srv);

  dparam_cli_.emplace(node, node_name_, ns);
}

void ParamBlockWidget::clearRosInterfaces()
{
  dparam_cli_.reset();
  get_param_sc_.reset();
}

bool ParamBlockWidget::load()
{
  clear();

  if (!get_param_sc_) {
    qt::qErrorBox(this, "ROS interfaces have not been initialized.");
    return false;
  }

  // Get dynamic parameters.
  const auto req = std::make_shared<tobas_dparam_msgs::srv::GetParams::Request>();
  const auto res = get_param_sc_->sendRequestAndWait(req);
  if (!res) {
    qt::qErrorBox(this, "Failed to get dynamic parameters configuration of \"" + label_->text() + "\".");
    return false;
  }
  const auto& params = res->params;

  // Add sliders.
  for (const auto& param : params.ints) {
    const auto param_name_label = new QLabel(QString::fromStdString(param.name));
    param_name_label->setFixedWidth(kParamNameWidth);

    IntConfig config;
    config.step = param.step;
    config.dflt = param.dflt;
    config.prefix = QString::fromStdString(str::convertToSuperscript(param.prefix));

    config.down_button_ = new QPushButton();
    config.down_button_->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));

    config.up_button_ = new QPushButton();
    config.up_button_->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));

    config.slider = new qt::Slider(Qt::Horizontal);
    config.slider->setRange(param.min, param.max);
    config.slider->setValue(param.value);

    config.line_edit = new QLineEdit();
    config.line_edit->setFixedWidth(kLineEditWidth);
    config.line_edit->setAlignment(Qt::AlignRight);
    config.line_edit->setReadOnly(true);
    config.line_edit->setText(QString::number(param.step * param.value) + config.prefix);

    int_configs_[param.name] = config;

    const auto cols = new QHBoxLayout();
    cols->addWidget(config.down_button_);
    cols->addWidget(config.up_button_);
    cols->addWidget(config.slider);
    cols->addWidget(config.line_edit);
    form_->addRow(param_name_label, cols);

    connect(config.down_button_, &QPushButton::clicked, std::bind(&self::onIntDownButtonClicked, this, param.name));
    connect(config.up_button_, &QPushButton::clicked, std::bind(&self::onIntUpButtonClicked, this, param.name));
    connect(config.slider, &qt::Slider::valueChanged, std::bind(&self::onIntSliderValueChanged, this, _1, param.name));
  }

  for (const auto& param : params.doubles) {
    const auto param_name_label = new QLabel(QString::fromStdString(param.name));
    param_name_label->setFixedWidth(kParamNameWidth);

    DoubleConfig config;
    config.step = param.step;
    config.dflt = param.dflt;
    config.prefix = QString::fromStdString(str::convertToSuperscript(param.prefix));

    config.down_button_ = new QPushButton();
    config.down_button_->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));

    config.up_button_ = new QPushButton();
    config.up_button_->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));

    config.slider = new qt::Slider(Qt::Horizontal);
    config.slider->setRange(param.min, param.max);
    config.slider->setValue(param.value);

    config.line_edit = new QLineEdit();
    config.line_edit->setFixedWidth(kLineEditWidth);
    config.line_edit->setAlignment(Qt::AlignRight);
    config.line_edit->setReadOnly(true);
    config.line_edit->setText(QString::number(param.step * param.value) + config.prefix);

    double_configs_[param.name] = config;

    const auto cols = new QHBoxLayout();
    cols->addWidget(config.down_button_);
    cols->addWidget(config.up_button_);
    cols->addWidget(config.slider);
    cols->addWidget(config.line_edit);
    form_->addRow(param_name_label, cols);

    connect(config.down_button_, &QPushButton::clicked, std::bind(&self::onDoubleDownButtonClicked, this, param.name));
    connect(config.up_button_, &QPushButton::clicked, std::bind(&self::onDoubleUpButtonClicked, this, param.name));
    connect(
      config.slider, &qt::Slider::valueChanged, std::bind(&self::onDoubleSliderValueChanged, this, _1, param.name));
  }

  return true;
}

bool ParamBlockWidget::save(const QString& path)
{
  const auto config = createCurrentConfig();

  // Confirm that the configuration file exists.
  if (!QFileInfo(path).isFile()) {
    qt::qErrorBox(this, path + " does not exist on PC.");
    return false;
  }

  // Save to the PC.
  if (!yaml::save(path.toStdString(), config)) {
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

bool ParamBlockWidget::setToDefaults()
{
  if (!dparam_cli_) {
    qt::qErrorBox(this, "ROS interfaces have not been initialized.");
    return false;
  }

  for (const auto& [name, config] : int_configs_) {
    if (config.slider->value() == config.dflt) {
      continue;
    }

    if (dparam_cli_->setInt(name, config.dflt) != dparam::DynamicParamClient::kNoError) {
      qWarning() << dparam_cli_->errorMessage();
      qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
      return false;
    }

    const QSignalBlocker block(config.slider);
    config.slider->setValue(config.dflt);
    config.line_edit->setText(QString::number(config.step * config.dflt) + config.prefix);
  }

  for (const auto& [name, config] : double_configs_) {
    if (config.slider->value() == config.dflt) {
      continue;
    }

    if (dparam_cli_->setDouble(name, config.dflt) != dparam::DynamicParamClient::kNoError) {
      qWarning() << dparam_cli_->errorMessage();
      qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
      return false;
    }

    const QSignalBlocker block(config.slider);
    config.slider->setValue(config.dflt);
    config.line_edit->setText(QString::number(config.step * config.dflt) + config.prefix);
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

void ParamBlockWidget::onIntDownButtonClicked(const std::string& name)
{
  auto& config = int_configs_.at(name);
  config.slider->setValue(config.slider->value() - 1);
}

void ParamBlockWidget::onIntUpButtonClicked(const std::string& name)
{
  auto& config = int_configs_.at(name);
  config.slider->setValue(config.slider->value() + 1);
}

void ParamBlockWidget::onIntSliderValueChanged(long value, const std::string& name)
{
  auto& config = int_configs_.at(name);
  config.line_edit->setText(QString::number(config.step * value) + config.prefix);

  if (dparam_cli_->setInt(name, value) != dparam::DynamicParamClient::kNoError) {
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
  }
}

void ParamBlockWidget::onDoubleDownButtonClicked(const std::string& name)
{
  auto& config = double_configs_.at(name);
  config.slider->setValue(config.slider->value() - 1);
}

void ParamBlockWidget::onDoubleUpButtonClicked(const std::string& name)
{
  auto& config = double_configs_.at(name);
  config.slider->setValue(config.slider->value() + 1);
}

void ParamBlockWidget::onDoubleSliderValueChanged(long value, const std::string& name)
{
  auto& config = double_configs_.at(name);
  config.line_edit->setText(QString::number(config.step * value) + config.prefix);

  if (dparam_cli_->setDouble(name, value) != dparam::DynamicParamClient::kNoError) {
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
  }
}
}  // namespace param
}  // namespace gui
}  // namespace tobas
