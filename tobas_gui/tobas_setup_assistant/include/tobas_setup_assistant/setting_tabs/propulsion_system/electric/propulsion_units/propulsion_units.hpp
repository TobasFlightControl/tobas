#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "./propulsion_unit.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class PropulsionUnitsWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = PropulsionUnitsWidget;
  using super = qt::TabWidget;

  static constexpr int kTabWidth = 150;
  static constexpr int kTabHeight = 50;

public:
  explicit PropulsionUnitsWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf);

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
  const rclcpp::Node::SharedPtr node_;
  const uadf::Model& uadf_;

private Q_SLOTS:
  void onCopyToAllButtonClicked(const QString& link_name);
};
};  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
