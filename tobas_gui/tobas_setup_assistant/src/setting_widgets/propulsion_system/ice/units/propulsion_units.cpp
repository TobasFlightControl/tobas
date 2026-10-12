// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/propulsion_units.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

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
PropulsionUnitsWidget::PropulsionUnitsWidget(const uadf::Model& _uadf) : uadf_(_uadf)
{
  setTabSize(150, 50);
}

void PropulsionUnitsWidget::updateInternalDataStructures()
{
  removeAllTabs();

  for (const auto& [joint_name, _] : uadf_.thrusts) {
    // Get propeller link names.
    const auto link_name = QString::fromStdString(uadf_.urdf->getJoint(joint_name)->child_link_name);

    // Add the tab.
    const auto link_widget = new PropulsionUnitWidget();
    addTab(link_widget, link_name);

    // Connection
    connect(
      link_widget,
      &PropulsionUnitWidget::copyToAllButtonClicked,
      this,
      std::bind(&self::onCopyToAllButtonClicked, this, link_name));
  }
}

void PropulsionUnitsWidget::setToDefaults()
{
  for (int i = 0; i < count(); ++i) {
    widget(i)->setToDefaults();
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

void PropulsionUnitsWidget::load(const YAML::Node& _node)
{
  for (const auto& pair : _node) {
    const auto link_name = pair.first.as<QString>();
    const auto& sub_node = pair.second;
    widget(link_name)->load(sub_node);
  }
}

int PropulsionUnitsWidget::numUnits() const
{
  return count();
}

QString PropulsionUnitsWidget::linkName(int _index) const
{
  return tabText(_index);
}

int PropulsionUnitsWidget::index(const QString& _link_name) const
{
  for (int i = 0; i < count(); ++i) {
    if (linkName(i) == _link_name) {
      return i;
    }
  }

  qWarning() << _link_name << "is not selected as a propulsion system.";
  return -1;
}

PropulsionUnitWidget* PropulsionUnitsWidget::widget(int _index)
{
  return qt::qPointerCast<PropulsionUnitWidget>(super::widget(_index));
}

const PropulsionUnitWidget* PropulsionUnitsWidget::widget(int _index) const
{
  return qt::qConstPointerCast<PropulsionUnitWidget>(super::widget(_index));
}

PropulsionUnitWidget* PropulsionUnitsWidget::widget(const QString& _link_name)
{
  return widget(index(_link_name));
}

const PropulsionUnitWidget* PropulsionUnitsWidget::widget(const QString& _link_name) const
{
  return widget(index(_link_name));
}

void PropulsionUnitsWidget::onCopyToAllButtonClicked(const QString& _link_name)
{
  const auto src_idx = index(_link_name);
  const auto src_widget = widget(src_idx);

  for (int dst_idx = 0; dst_idx < count(); ++dst_idx) {
    if (dst_idx == src_idx) {
      continue;
    }
    const auto dst_widget = widget(dst_idx);
    dst_widget->copyFrom(src_widget);
  }

  qt::qInfoBox(this, "The settings of '" + _link_name + "' have been copied to all the other propulsion units.");
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
