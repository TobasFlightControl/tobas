// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../base_setting.hpp"
#include "./network_iface.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace network
{
class NetworkWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = NetworkWidget;
  using super = BaseSettingWidget;

public:
  explicit NetworkWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  QString networkInterface() const;

private:
  NetworkIfaceWidget* nic_;
};
}  // namespace network
}  // namespace sa
}  // namespace gui
}  // namespace tobas
