// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QPushButton>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "tobas_setup_assistant/signals.hpp"

namespace tobas::gui::sa::hw
{
class DShotWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = DShotWidget;
  using super = qt::TableWidget;

public:
  explicit DShotWidget(const uadf::Model& _uadf, const Signals& _sig);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  void setNumChannels(int _num);

  QString targetName(int _channel) const;
  bool bidirectional(int _channel) const;

  bool contains(const QString& _target_name) const;
  int channel(const QString& _target_name) const;

private:
  const uadf::Model& uadf_;

  PropulsionSystem prop_type_ = PropulsionSystem::kElectric;

  qt::ComboBox* targetNameWidget(int _row);
  QPushButton* bidirectionalWidget(int _row);

  const qt::ComboBox* targetNameWidget(int _row) const;
  const QPushButton* bidirectionalWidget(int _row) const;

  void addLastChannel();
  void removeLastChannel();

  void setBidirectionalButtonChecked(QPushButton* _button, bool _checked);
  void setBidirectionalButtonText(QPushButton* _button, bool _checked);

private Q_SLOTS:
  void onPropulsionTypeChanged(PropulsionSystem _new_prop_type);
  void onBidirectionalButtonToggled(QPushButton* _button, bool _checked);
};
}  // namespace tobas::gui::sa::hw
