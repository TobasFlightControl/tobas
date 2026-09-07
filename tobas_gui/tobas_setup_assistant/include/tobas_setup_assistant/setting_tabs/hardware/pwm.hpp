// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "tobas_setup_assistant/signals.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace hw
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

  explicit PwmWidget(const uadf::Model& uadf, const Signals& sig);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  void setNumChannels(int num);

  QString targetName(int channel) const;
  TargetType targetType(int channel) const;

  double periodLb(int channel) const;  // [us]
  double periodUb(int channel) const;  // [us]

  bool contains(const QString& target_name) const;
  int channel(const QString& target_name) const;

private:
  const uadf::Model& uadf_;

  PropulsionSystem prop_type_ = PropulsionSystem::kElectric;

  qt::ComboBox* targetNameWidget(int row);
  qt::DoubleSpinBox* periodLbWidget(int row);
  qt::DoubleSpinBox* periodUbWidget(int row);

  const qt::ComboBox* targetNameWidget(int row) const;
  const qt::DoubleSpinBox* periodLbWidget(int row) const;
  const qt::DoubleSpinBox* periodUbWidget(int row) const;

  void addLastChannel();
  void removeLastChannel();

private Q_SLOTS:
  void onPropulsionTypeChanged(const PropulsionSystem& new_prop_type);
};
}  // namespace hw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
