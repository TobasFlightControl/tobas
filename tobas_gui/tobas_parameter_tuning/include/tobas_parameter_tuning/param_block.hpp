// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

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
  long default_value;
  long initial_value;
  QString prefix;

  QPushButton* down_button_;
  QPushButton* up_button_;
  QPushButton* reset_button_;
  QPushButton* home_button_;
  qt::Slider* slider;
  QLineEdit* line_edit;
};

struct DoubleConfig
{
  double step;
  long default_value;
  long initial_value;
  QString prefix;

  QPushButton* down_button_;
  QPushButton* up_button_;
  QPushButton* reset_button_;
  QPushButton* home_button_;
  qt::Slider* slider;
  QLineEdit* line_edit;
};

class ParamBlockWidget : public QWidget
{
  Q_OBJECT

  using self = ParamBlockWidget;

public:
  explicit ParamBlockWidget(const std::string& node_name, const QString& label);

  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

  bool load();
  bool save(const QString& path);
  void clear();

  bool setToInitialValues();
  bool setToDefaultValues();

private:
  const std::string node_name_;

  std::optional<ros2::SyncServiceClient<tobas_dparam_msgs::srv::GetParams>> get_param_sc_;
  std::optional<dparam::DynamicParamClient> dparam_cli_;

  std::map<std::string, IntConfig> int_configs_;
  std::map<std::string, DoubleConfig> double_configs_;

  QLabel* label_;
  qt::FormLayout* form_;

  YAML::Node createCurrentConfig() const;

  template <typename Config>
  bool setValue(const std::string& name, const Config& config, int32_t value);

private Q_SLOTS:
  void onIntDownButtonClicked(const std::string& name);
  void onIntUpButtonClicked(const std::string& name);
  void onIntResetButtonClicked(const std::string& name);
  void onIntHomeButtonClicked(const std::string& name);
  void onIntSliderValueChanged(long value, const std::string& name);

  void onDoubleDownButtonClicked(const std::string& name);
  void onDoubleUpButtonClicked(const std::string& name);
  void onDoubleResetButtonClicked(const std::string& name);
  void onDoubleHomeButtonClicked(const std::string& name);
  void onDoubleSliderValueChanged(long value, const std::string& name);
};
}  // namespace param
}  // namespace gui
}  // namespace tobas
