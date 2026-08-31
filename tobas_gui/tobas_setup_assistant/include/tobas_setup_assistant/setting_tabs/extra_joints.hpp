// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QPushButton>

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

  using self = ExtraJointsWidget;
  using super = BaseSettingWidget;

public:
  explicit ExtraJointsWidget(const uadf::Model& uadf, const kdl::Tree& tree);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  // Getters
  QString getLinkName(int row) const;
  QString getJointName(int row) const;
  JointRole getRole(int row) const;
  JointCommandInterface getCommandInterface(int row) const;
  double getHomePosition(int row) const;  // [rad]

  // Setters
  void setCommandInterface(int row, JointCommandInterface value);
  void setHomePosition(int row, double value);  // [rad]

  /* Number of registered joints. */
  int numJoints() const;

  /* Return the table row for the link name, or -1 if it does not exist. */
  int findLink(const QString& link_name) const;

  /* Return the table row for the joint name, or -1 if it does not exist. */
  int findJoint(const QString& joint_name) const;

private:
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;

  qt::TableWidget* table_;

  QLabel* linkNameWidget(int row);
  const QLabel* linkNameWidget(int row) const;

  QLabel* jointNameWidget(int row);
  const QLabel* jointNameWidget(int row) const;

  qt::ComboBox* commandIfaceWidget(int row);
  const qt::ComboBox* commandIfaceWidget(int row) const;

  qt::SpinBox* homePositionWidget(int row);
  const qt::SpinBox* homePositionWidget(int row) const;

  void clear();
  void reset(int row);
  void addLink(const std::string& link_name);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
