// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>

#include <tobas_drone_core/joint/command_interface.hpp>
#include <tobas_drone_core/joint/role.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "./base_setting.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ExtraJointsWidget : public BaseSettingWidget
{
  Q_OBJECT

public:
  explicit ExtraJointsWidget(const uadf::Model& _uadf, const kdl::Tree& _tree);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  // Getters
  QString getLinkName(int _row) const;
  QString getJointName(int _row) const;
  JointRole getRole(int _row) const;
  JointCommandInterface getCommandInterface(int _row) const;
  double getHomePosition(int _row) const;  // [rad]

  // Setters
  void setCommandInterface(int _row, JointCommandInterface _value);
  void setHomePosition(int _row, double _value);  // [rad]

  /** Number of registered joints. */
  int numJoints() const;

  /** Return the table row for the link name, or -1 if it does not exist. */
  int findLink(const QString& _link_name) const;

  /** Return the table row for the joint name, or -1 if it does not exist. */
  int findJoint(const QString& _joint_name) const;

private:
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;

  qt::TableWidget* table_;

  QLabel* linkNameWidget(int _row);
  const QLabel* linkNameWidget(int _row) const;

  QLabel* jointNameWidget(int _row);
  const QLabel* jointNameWidget(int _row) const;

  qt::ComboBox* commandIfaceWidget(int _row);
  const qt::ComboBox* commandIfaceWidget(int _row) const;

  qt::SpinBox* homePositionWidget(int _row);
  const qt::SpinBox* homePositionWidget(int _row) const;

  void clear();
  void reset(int _row);
  void addLink(const std::string& _link_name);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
