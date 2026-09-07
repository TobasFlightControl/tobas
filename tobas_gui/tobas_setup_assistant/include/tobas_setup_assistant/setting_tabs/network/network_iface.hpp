// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QButtonGroup>
#include <QLineEdit>
#include <QRadioButton>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace network
{
class NetworkIfaceWidget : public QWidget
{
  Q_OBJECT

  using self = NetworkIfaceWidget;
  using super = QWidget;

public:
  explicit NetworkIfaceWidget();

  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  QString networkInterface() const;

private:
  QButtonGroup* nic_btn_group_;
  QLineEdit* other_nic_name_;

  QRadioButton* addNicTypeButton(const QString& text, int id);

private Q_SLOTS:
  void onOtherButtonToggled(bool checked);
};
}  // namespace network
}  // namespace sa
}  // namespace gui
}  // namespace tobas
