// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace mission
{
class BaseExecutorWidget : public QWidget
{
  Q_OBJECT

public:
  virtual QString executorPackage() const = 0;
  virtual QString pluginName() const = 0;

  /* 静的プライベートROSパラメータ． */
  virtual YAML::Node staticParams() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* ユーザ設定が有効な場合にtrueを返す． */
  virtual bool isValid() = 0;
};
}  // namespace mission
}  // namespace sa
}  // namespace gui
}  // namespace tobas
