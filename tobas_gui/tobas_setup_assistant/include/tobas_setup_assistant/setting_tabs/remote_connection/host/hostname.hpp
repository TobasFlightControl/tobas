// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace rc
{
class HostnameWidget : public BaseHostWidget
{
  Q_OBJECT

  static constexpr char kHostnameKey[] = "hostname";

public:
  explicit HostnameWidget();

  void setToDefaults() override;
  const char* label() const override;

  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  QString host() const override;

private:
  QLineEdit* hostname_;
};
}  // namespace rc
}  // namespace sa
}  // namespace gui
}  // namespace tobas
