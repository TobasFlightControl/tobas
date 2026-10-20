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
#include <tobas_rqt_tools/sync_service_client.hpp>

#include <tobas_dparam_msgs/srv/get_params.hpp>

namespace tobas::gui::param
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
  explicit ParamBlockWidget(const std::string& _node_name, const QString& _label);

  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

  QString getLabelText() const;

  bool load();
  bool save(const QString& _path);
  void clear();

  bool setToInitialValues();
  bool setToDefaultValues();

private:
  const std::string node_name_;

  std::optional<rqt::SyncServiceClientWrapper<tobas_dparam_msgs::srv::GetParams>> get_param_sc_;
  std::optional<dparam::DynamicParamClient> dparam_cli_;

  std::map<std::string, IntConfig> int_configs_;
  std::map<std::string, DoubleConfig> double_configs_;

  QLabel* label_;
  qt::FormLayout* form_;

  YAML::Node createCurrentConfig() const;

  template <typename Config>
  bool setValue(const std::string& _name, const Config& _config, int32_t _value);

private Q_SLOTS:
  void onIntDownButtonClicked(const std::string& _name);
  void onIntUpButtonClicked(const std::string& _name);
  void onIntResetButtonClicked(const std::string& _name);
  void onIntHomeButtonClicked(const std::string& _name);
  void onIntSliderValueChanged(long _value, const std::string& _name);

  void onDoubleDownButtonClicked(const std::string& _name);
  void onDoubleUpButtonClicked(const std::string& _name);
  void onDoubleResetButtonClicked(const std::string& _name);
  void onDoubleHomeButtonClicked(const std::string& _name);
  void onDoubleSliderValueChanged(long _value, const std::string& _name);
};
}  // namespace tobas::gui::param
