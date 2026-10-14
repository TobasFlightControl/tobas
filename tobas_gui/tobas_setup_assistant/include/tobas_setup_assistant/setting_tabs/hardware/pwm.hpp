// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "tobas_setup_assistant/signals.hpp"

namespace tobas::gui::sa::hw
{
class PwmWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = PwmWidget;
  using super = qt::TableWidget;

public:
  // Special target labels
  static constexpr char kEngineThrotLabel[] = "Engine Throttle";

  enum class TargetType
  {
    kThrust,
    kControlSurface,
    kTiltJoint,
    kEngineThrottle,
  };

  explicit PwmWidget(const uadf::Model& _uadf, const Signals& _sig);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  void setNumChannels(int _num);

  QString targetName(int _channel) const;
  TargetType targetType(int _channel) const;

  double periodLb(int _channel) const;  // [us]
  double periodUb(int _channel) const;  // [us]

  bool contains(const QString& _target_name) const;
  int channel(const QString& _target_name) const;

private:
  const uadf::Model& uadf_;

  PropulsionSystem prop_type_ = PropulsionSystem::kElectric;

  qt::ComboBox* targetNameWidget(int _row);
  qt::DoubleSpinBox* periodLbWidget(int _row);
  qt::DoubleSpinBox* periodUbWidget(int _row);

  const qt::ComboBox* targetNameWidget(int _row) const;
  const qt::DoubleSpinBox* periodLbWidget(int _row) const;
  const qt::DoubleSpinBox* periodUbWidget(int _row) const;

  void addLastChannel();
  void removeLastChannel();

private Q_SLOTS:
  void onPropulsionTypeChanged(PropulsionSystem _new_prop_type);
};
}  // namespace tobas::gui::sa::hw
