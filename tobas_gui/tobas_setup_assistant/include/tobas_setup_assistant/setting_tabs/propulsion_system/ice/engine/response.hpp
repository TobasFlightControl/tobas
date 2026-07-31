// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include "tobas_setup_assistant/param_getters/spin_box.hpp"

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

  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  /* [s] */
  double timeConstUp() const;

  /* [s] */
  double timeConstDown() const;

private:
  ParamGetterWidget_SpinBox* time_const_up_;
  ParamGetterWidget_SpinBox* time_const_down_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
