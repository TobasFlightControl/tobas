#include <QVBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_yaml_tools/core.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_dparam_msgs/srv/get_params.hpp>

#include "tobas_parameter_tuning_gui/param_block.hpp"

using namespace std;
namespace fs = filesystem;

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

bool ParamBlockWidget::load(const string& ns, const string& node_name)
{
  node_name_ = node_name;
  dparam_client_ = make_shared<dparam::DynamicParamClient>(node_, node_name, ns);

  clear();

  // Get dynamic parameters
  ros2::SyncServiceClient<tobas_dparam_msgs::srv::GetParams> sc(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, node_name, tobas::kGetDynamicParamsSrv));
  const auto req = make_shared<tobas_dparam_msgs::srv::GetParams::Request>();
  if (!sc.call(req, kLoadParamTimeout))
  {
    qt::qErrorBox(this, "Failed to get dynamic parameters configuration of \"" + label_->text() + "\".");
    return false;
  }
  const auto res = sc.getResponse();
  const auto& params = res->params;

  // Add sliders
  for (const auto& param : params.ints)
  {
    IntConfig config;
    config.slider = new qt::IntSliderTextWidget(param.min, param.max);
    config.slider->set(param.value);
    config.dflt = param.dflt;

    connect(
      config.slider, &qt::IntSliderTextWidget::valueChanged,
      bind(&self::onIntParamChanged, this, placeholders::_1, param.name));

    int_configs_[param.name] = config;
    form_->addRow(param.name.c_str(), config.slider);
  }

  for (const auto& param : params.doubles)
  {
    DoubleConfig config;
    config.slider = new qt::DoubleSliderTextWidget(param.min, param.max);
    config.slider->set(param.value);
    config.dflt = param.dflt;

    connect(
      config.slider, &qt::DoubleSliderTextWidget::valueChanged,
      bind(&self::onDoubleParamChanged, this, placeholders::_1, param.name));

    double_configs_[param.name] = config;
    form_->addRow(param.name.c_str(), config.slider);
  }

  return true;
}

bool ParamBlockWidget::save(const fs::path& local_path, const fs::path& remote_path)
{
  const auto config = createCurrentConfig();

  if (!saveRemote(remote_path, config))
    return false;

  if (!saveLocal(local_path, config))
    return false;

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
  for (const auto& [name, config] : int_configs_)
  {
    if (config.slider->get() == config.dflt)
      continue;

    if (dparam_client_->set(name, config.dflt) != dparam::DynamicParamClient::E_NO_ERROR)
    {
      qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
      return false;
    }

    config.slider->blockSignals(true);
    config.slider->set(config.dflt);
    config.slider->blockSignals(false);
  }

  for (const auto& [name, config] : double_configs_)
  {
    if (config.slider->get() == config.dflt)
      continue;

    if (dparam_client_->set(name, config.dflt) != dparam::DynamicParamClient::E_NO_ERROR)
    {
      qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
      return false;
    }

    config.slider->blockSignals(true);
    config.slider->set(config.dflt);
    config.slider->blockSignals(false);
  }

  return true;
}

YAML::Node ParamBlockWidget::createCurrentConfig() const
{
  YAML::Node res(YAML::NodeType::Map);

  for (const auto& [name, config] : int_configs_)
    res[name] = config.slider->get();

  for (const auto& [name, config] : double_configs_)
    res[name] = format("{:e}", config.slider->get());  // 整数に丸められるとrosparamが取得できないため指数表記を強制

  return res;
}

bool ParamBlockWidget::saveLocal(const fs::path& path, const YAML::Node& node)
{
  // 設定ファイルが存在することを確認
  if (!fs::is_regular_file(path))
  {
    qt::qErrorBox(this, QString::fromStdString(path) + " does not exist on PC.");
    return false;
  }

  // PCに保存
  if (!yaml::save(path, node))
  {
    qt::qErrorBox(this, "Failed to save configuration to PC.");
    return false;
  }

  return true;
}

bool ParamBlockWidget::saveRemote(const fs::path& path, const YAML::Node& node)
{
  // SSH接続を確認
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return false;
  }

  // 設定ファイルが存在することを確認
  if (!ssh_client_.fileExists(path))
  {
    qt::qErrorBox(this, QString::fromStdString(path) + " does not exist on FC.");
    return false;
  }

  // FCに書き込む
  const auto config_text = yaml::dump(node);
  if (ssh_client_.sftpWrite(path, config_text, true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "Failed to save configuration to FC: " + QString(ssh_client_.errorMessage()));
    return false;
  }

  return true;
}

void ParamBlockWidget::onIntParamChanged(int value, const string& name)
{
  if (dparam_client_->set(name, value) != dparam::DynamicParamClient::E_NO_ERROR)
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
}

void ParamBlockWidget::onDoubleParamChanged(double value, const string& name)
{
  if (dparam_client_->set(name, value) != dparam::DynamicParamClient::E_NO_ERROR)
    qt::qErrorBox(this, "Failed to set " + label_->text() + "'s parameter \"" + name.c_str() + "\".");
}
}  // namespace param
}  // namespace gui
