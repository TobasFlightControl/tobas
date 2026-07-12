// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <yaml-cpp/yaml.h>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_dparam_client/dparam_client.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/slider.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_dparam_msgs/srv/get_params.hpp>

namespace tobas
{
namespace gui
{
namespace param
{
struct IntConfig
{
  long step;
  long dflt;
  QString prefix;

  QPushButton* down_button_;
  QPushButton* up_button_;
  qt::Slider* slider;
  QLineEdit* line_edit;
};

struct DoubleConfig
{
  double step;
  long dflt;
  QString prefix;

  QPushButton* down_button_;
  QPushButton* up_button_;
  qt::Slider* slider;
  QLineEdit* line_edit;
};

class ParamBlockWidget : public QWidget
{
  Q_OBJECT

  using self = ParamBlockWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kParamNameWidth = 250;
  static constexpr int kLineEditWidth = 150;

public:
  explicit ParamBlockWidget(rclcpp::Node::SharedPtr node, const std::string& node_name, const QString& label);

  void setNamespace(const std::string& ns);

  bool load();
  bool save(const std::filesystem::path& path);
  void clear();
  bool setToDefaults();

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string node_name_;

  ros2::SyncServiceClient<tobas_dparam_msgs::srv::GetParams>::SharedPtr get_param_sc_;
  dparam::DynamicParamClient::SharedPtr dparam_cli_;

  std::map<std::string, IntConfig> int_configs_;
  std::map<std::string, DoubleConfig> double_configs_;

  QLabel* label_;
  qt::FormLayout* form_;

  YAML::Node createCurrentConfig() const;

private Q_SLOTS:
  void onIntDownButtonClicked(const std::string& name);
  void onIntUpButtonClicked(const std::string& name);
  void onIntSliderValueChanged(long value, const std::string& name);

  void onDoubleDownButtonClicked(const std::string& name);
  void onDoubleUpButtonClicked(const std::string& name);
  void onDoubleSliderValueChanged(long value, const std::string& name);
};
}  // namespace param
}  // namespace gui
}  // namespace tobas
