#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>

#include "./propulsion_unit.hpp"
#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class PropulsionUnitsWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = PropulsionUnitsWidget;
  using super = qt::TabWidget;

  static constexpr int kTabWidth = 150;
  static constexpr int kTabHeight = 50;

public:
  explicit PropulsionUnitsWidget(const RobotInfo& robot);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  int numUnits() const;

  QString linkName(int index) const;

  /* タブのインデックスを返す．存在しなければ-1を返す． */
  int index(const QString& link_name) const;

  PropulsionUnitWidget* widget(int index);
  const PropulsionUnitWidget* widget(int index) const;
  PropulsionUnitWidget* widget(const QString& link_name);
  const PropulsionUnitWidget* widget(const QString& link_name) const;

private:
  const RobotInfo& robot_;

private Q_SLOTS:
  void onCopyFromLeftButtonClicked(const QString& link_name);
  void onCopyToAllButtonClicked(const QString& link_name);
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
