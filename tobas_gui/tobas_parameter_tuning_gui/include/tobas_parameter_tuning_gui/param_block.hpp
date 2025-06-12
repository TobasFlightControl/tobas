#pragma once

#include <filesystem>

#include <yaml-cpp/yaml.h>
#include <QLabel>
#include <QLineEdit>

#include <tobas_dparam_client/dparam_client.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/slider.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ssh_client/ssh_client.hpp>

namespace gui
{
namespace param
{
struct IntConfig
{
  long dflt;
  QString prefix;

  qt::Slider* slider;
  QLineEdit* line_edit;
};

struct DoubleConfig
{
  double step;
  long dflt;
  QString prefix;

  qt::Slider* slider;
  QLineEdit* line_edit;
};

class ParamBlockWidget : public QWidget
{
  Q_OBJECT

  using self = ParamBlockWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kLineEditWidth = 100;
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
  void onIntParamChanged(long value, const std::string& name);
  void onDoubleParamChanged(long value, const std::string& name);
};
}  // namespace param
}  // namespace gui
