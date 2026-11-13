// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas::gui::sa::mission
{
class NoneWidget : public BaseExecutorWidget
{
  Q_OBJECT

public:
  explicit NoneWidget();

  void setToDefaults() override;

  QString executorPackage() const override;
  QString pluginName() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  bool isValid() override;
};
}  // namespace tobas::gui::sa::mission
