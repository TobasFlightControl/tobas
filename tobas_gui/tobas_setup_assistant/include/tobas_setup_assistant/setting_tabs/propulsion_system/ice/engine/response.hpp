// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineResponseWidget : public QWidget
{
  Q_OBJECT

  using self = EngineResponseWidget;
  using super = QWidget;

public:
  explicit EngineResponseWidget();

  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  /* [s] */
  double timeConstUp() const;

  /* [s] */
  double timeConstDown() const;

private:
  qt::SpinBox* time_const_up_;
  qt::SpinBox* time_const_down_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
