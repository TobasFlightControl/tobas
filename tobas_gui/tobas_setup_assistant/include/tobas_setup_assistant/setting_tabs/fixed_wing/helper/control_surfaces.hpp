// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "./wings.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
namespace hp
{
class ControlSurfacesWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = ControlSurfacesWidget;
  using super = qt::TableWidget;

  static constexpr int kColWidth = 120;
  static constexpr double kAngleLimit = M_PI_4;

  // Columns
  static constexpr int kLinkNameCol = 0;
  static constexpr int kJointNameCol = kLinkNameCol + 1;
  static constexpr int kWingCol = kJointNameCol + 1;
  static constexpr int kStartSpanCol = kWingCol + 1;
  static constexpr int kFinishSpanCol = kStartSpanCol + 1;
  static constexpr int kChordRatioCol = kFinishSpanCol + 1;
  static constexpr int kNumCols = kChordRatioCol + 1;

  // Labels
  static constexpr char kLinkNameLabel[] = "Link Name";
  static constexpr char kJointNameLabel[] = "Joint Name";
  static constexpr char kWingLabel[] = "Wing Index";
  static constexpr char kStartSpanLabel[] = "Start Span";
  static constexpr char kFinishSpanLabel[] = "Finish Span";
  static constexpr char kChordRatioLabel[] = "Chord Ratio";

public:
  explicit ControlSurfacesWidget(const uadf::Model& uadf, WingsWidget* wings_widget);

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  void add(const QString& link_name);

  // Getters
  QString linkName(int row) const;
  QString jointName(int row) const;
  int wingIdx(int row) const;
  double startSpan(int row) const;
  double finishSpan(int row) const;
  double chordRatio(int row) const;

  // Setters
  void linkName(int row, const QString& text);
  void jointName(int row, const QString& text);
  void wingIdx(int row, int index);
  void startSpan(int row, double value);
  void finishSpan(int row, double value);
  void chordRatio(int row, double value);

private:
  const uadf::Model& uadf_;
  WingsWidget* wings_widget_;

  void setToDefault(int row);
  int find(const QString& link_name) const;
  void onWingsTabAdded(QString tab_name);
  void onWingsTabRemoved(int index);
  void onWingsTabRenamed(int index, QString new_name);
};
}  // namespace hp
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
