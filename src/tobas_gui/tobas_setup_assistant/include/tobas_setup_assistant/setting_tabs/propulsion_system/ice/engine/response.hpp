// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace tobas::gui::sa::propulsion::ice
{
class EngineResponseWidget : public QWidget
{
  Q_OBJECT

public:
  explicit EngineResponseWidget();

  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  /** [s] */
  double timeConstUp() const;

  /** [s] */
  double timeConstDown() const;

private:
  qt::SpinBox* time_const_up_;
  qt::SpinBox* time_const_down_;
};
}  // namespace tobas::gui::sa::propulsion::ice
