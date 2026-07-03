// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/propulsion_units.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/stream.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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
PropulsionUnitsWidget::PropulsionUnitsWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf)
  : node_(node), uadf_(uadf)
{
  setTabSize(kTabWidth, kTabHeight);
}

void PropulsionUnitsWidget::updateInternalDataStructures()
{
  removeAllTabs();

  for (const auto& [joint_name, _] : uadf_.thrusts) {
    // Get propeller link names.
    const auto link_name = QString::fromStdString(uadf_.urdf->getJoint(joint_name)->child_link_name);

    // Add the tab.
    const auto link_widget = new PropulsionUnitWidget(node_);
    addTab(link_widget, link_name);

    // Connection
    connect(
      link_widget,
      &PropulsionUnitWidget::copyToAllButtonClicked,
      this,
      std::bind(&self::onCopyToAllButtonClicked, this, link_name));
  }
}

bool PropulsionUnitsWidget::isValid()
{
  for (int i = 0; i < count(); ++i) {
    if (!widget(i)->isValid()) {
      return false;
    }
  }

  return true;
}

YAML::Node PropulsionUnitsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int i = 0; i < count(); ++i) {
    const auto link_name = linkName(i);
    node[link_name] = widget(i)->dump();
  }

  return node;
}

void PropulsionUnitsWidget::load(const YAML::Node& node)
{
  for (const auto& pair : node) {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;
    widget(link_name)->load(sub_node);
  }
}

int PropulsionUnitsWidget::numUnits() const
{
  return count();
}

QString PropulsionUnitsWidget::linkName(int index) const
{
  return tabText(index);
}

int PropulsionUnitsWidget::index(const QString& link_name) const
{
  for (int i = 0; i < count(); ++i) {
    if (linkName(i) == link_name) {
      return i;
    }
  }

  qWarning() << link_name << "is not selected as a propulsion system.";
  return -1;
}

PropulsionUnitWidget* PropulsionUnitsWidget::widget(int index)
{
  return qt::qPointerCast<PropulsionUnitWidget>(super::widget(index));
}

const PropulsionUnitWidget* PropulsionUnitsWidget::widget(int index) const
{
  return qt::qConstPointerCast<PropulsionUnitWidget>(super::widget(index));
}

PropulsionUnitWidget* PropulsionUnitsWidget::widget(const QString& link_name)
{
  return widget(index(link_name));
}

const PropulsionUnitWidget* PropulsionUnitsWidget::widget(const QString& link_name) const
{
  return widget(index(link_name));
}

void PropulsionUnitsWidget::onCopyToAllButtonClicked(const QString& link_name)
{
  const auto src_idx = index(link_name);
  const auto src_widget = widget(src_idx);

  for (int dst_idx = 0; dst_idx < count(); ++dst_idx) {
    if (dst_idx == src_idx) {
      continue;
    }
    const auto dst_widget = widget(dst_idx);
    dst_widget->copyFrom(src_widget);
  }

  qt::qInfoBox(this, "The settings of \"" + link_name + "\" have been copied to all the other propulsion units.");
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
