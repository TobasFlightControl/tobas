// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>

#include "./base_setting.hpp"

namespace tobas::gui::sa
{
class MaintainerInformationWidget : public BaseSettingWidget
{
  Q_OBJECT

public:
  explicit MaintainerInformationWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  QString maintainerName() const;
  QString maintainerEmail() const;

private:
  QLineEdit* name_;
  QLineEdit* email_;
};
}  // namespace tobas::gui::sa
