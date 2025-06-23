#include "tobas_parameter_tuning_gui/param_block.hpp"

#include <QHBoxLayout>
#include <QStyle>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_yaml_tools/core.hpp>

#include <tobas_dparam_msgs/srv/get_params.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace param
{
ParamBlockWidget::ParamBlockWidget(rclcpp::Node::SharedPtr node, const QString& label) : node_(node), ssh_client_(node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  label_ = new QLabel(label);
  label_->setFont(qt::DefaultFont(kLabelPSize, QFont::Bold));
  qt::addWidgetCenter(label_, rows);

  form_ = new qt::FormLayout();
  rows->addLayout(form_);
}

bool ParamBlockWidget::load(const std::string& ns, const std::string& node_name)
{
  node_name_ = node_name;
  dparam_client_ = make_shared<dparam::DynamicParamClient>(node_, node_name, ns);

  clear();

  // Get dynamic parameters
  ros2::SyncServiceClient<tobas_dparam_msgs::srv::GetParams> sc(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, node_name, tobas::kGetDynamicParamsSrv));
  const auto req = std::make_shared<tobas_dparam_msgs::srv::GetParams::Request>();
  if (!sc.call(req, kLoadParamTimeout)) {
    qt::qErrorBox(this, "Failed to get dynamic parameters configuration of \"" + label_->text() + "\".");
    return false;
  }
  const auto res = sc.getResponse();
  const auto& params = res->params;

  // Add sliders
  for (const auto& param : params.ints) {
    const auto param_name_label = new QLabel(QString::fromStdString(param.name));
    param_name_label->setFixedWidth(kParamNameWidth);

    IntConfig config;
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
    config.line_edit->setText(QString::number(param.value) + config.prefix);

    int_configs_[param.name] = config;

    const auto cols = new QHBoxLayout();
    cols->addWidget(config.down_button_);
    cols->addWidget(config.up_button_);
    cols->addWidget(config.slider);
    cols->addWidget(config.line_edit);
    form_->addRow(param_name_label, cols);

    connect(config.down_button_, &QPushButton::clicked, bind(&self::onIntDownButtonClicked, this, param.name));
    connect(config.up_button_, &QPushButton::clicked, bind(&self::onIntUpButtonClicked, this, param.name));
    connect(
      config.slider,
      &qt::Slider::valueChanged,
      std::bind(&self::onIntSliderValueChanged, this, std::placeholders::_1, param.name));
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

    connect(config.down_button_, &QPushButton::clicked, bind(&self::onDoubleDownButtonClicked, this, param.name));
    connect(config.up_button_, &QPushButton::clicked, bind(&self::onDoubleUpButtonClicked, this, param.name));
    connect(
      config.slider,
      &qt::Slider::valueChanged,
      std::bind(&self::onDoubleSliderValueChanged, this, std::placeholders::_1, param.name));
  }

  return true;
}

bool ParamBlockWidget::saveLocal(const fs::path& path)
{
  const auto config = createCurrentConfig();

  // 設定ファイルが存在することを確認
  if (!fs::is_regular_file(path)) {
    qt::qErrorBox(this, QString::fromStdString(path) + " does not exist on PC.");
    return false;
  }

  // PCに保存
  if (!yaml::save(path, config)) {
    qt::qErrorBox(this, "Failed to save configuration to PC.");
    return false;
  }

  return true;
}

bool ParamBlockWidget::saveRemote(const fs::path& path)
{
  const auto config = createCurrentConfig();

  // 設定ファイルが存在することを確認
  if (!ssh_client_.fileExists(path)) {
    qt::qErrorBox(this, QString::fromStdString(path) + " does not exist on FC.");
    return false;
  }

  // FCに書き込む
  const auto config_text = yaml::dump(config);
  if (ssh_client_.sftpWrite(path, config_text, true) != ssh::SSHClient::E_NO_ERROR) {
    qt::qErrorBox(this, "Failed to save configuration to FC: " + QString(ssh_client_.errorMessage()));
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
  for (const auto& [name, config] : int_configs_) {
    if (config.slider->value() == config.dflt) {
      continue;
    }

    if (dparam_client_->setInt(name, config.dflt) != dparam::DynamicParamClient::E_NO_ERROR) {
      qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
      return false;
    }

    config.slider->blockSignals(true);
    config.slider->setValue(config.dflt);
    config.line_edit->setText(QString::number(config.dflt) + config.prefix);
    config.slider->blockSignals(false);
  }

  for (const auto& [name, config] : double_configs_) {
    if (config.slider->value() == config.dflt) {
      continue;
    }

    if (dparam_client_->setDouble(name, config.dflt) != dparam::DynamicParamClient::E_NO_ERROR) {
      qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
      return false;
    }

    config.slider->blockSignals(true);
    config.slider->setValue(config.dflt);
    config.line_edit->setText(QString::number(config.step * config.dflt) + config.prefix);
    config.slider->blockSignals(false);
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
  config.line_edit->setText(QString::number(value) + config.prefix);

  if (dparam_client_->setInt(name, value) != dparam::DynamicParamClient::E_NO_ERROR) {
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

  if (dparam_client_->setDouble(name, value) != dparam::DynamicParamClient::E_NO_ERROR) {
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
  }
}
}  // namespace param
}  // namespace gui
