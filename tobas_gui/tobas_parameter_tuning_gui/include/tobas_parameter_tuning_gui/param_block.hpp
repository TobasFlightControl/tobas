#pragma once

#include <filesystem>

#include <yaml-cpp/yaml.h>
#include <QWidget>
#include <QLabel>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_ssh_client/ssh_client.hpp>
#include <tobas_dparam_client/dparam_client.hpp>
#include <tobas_qt_tools/widgets/slider_text.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>

namespace gui
{
namespace param
{
struct IntConfig
{
  int dflt;
  qt::IntSliderTextWidget* slider;
};

struct DoubleConfig
{
  double dflt;
  qt::DoubleSliderTextWidget* slider;
};

class ParamBlockWidget : public QWidget
{
  Q_OBJECT

  using self = ParamBlockWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr auto kLoadParamTimeout = std::chrono::seconds(3);

public:
  explicit ParamBlockWidget(rclcpp::Node::SharedPtr node, const QString& label);

  bool load(const std::string& ns, const std::string& node_name);
  bool save(const std::filesystem::path& local_path, const std::filesystem::path& remote_path);

  void clear();

  bool setToDefaults();

private:
  const rclcpp::Node::SharedPtr node_;
  ssh::SSHClient ssh_client_;
  dparam::DynamicParamClient::SharedPtr dparam_client_;

  std::string node_name_;
  std::map<std::string, IntConfig> int_configs_;
  std::map<std::string, DoubleConfig> double_configs_;

  QLabel* label_;
  qt::FormLayout* form_;

  YAML::Node createCurrentConfig() const;

  bool saveLocal(const std::filesystem::path& path, const YAML::Node& node);
  bool saveRemote(const std::filesystem::path& path, const YAML::Node& node);

private Q_SLOTS:
  void onIntParamChanged(int value, const std::string& name);
  void onDoubleParamChanged(double value, const std::string& name);
};
}  // namespace param
}  // namespace gui
