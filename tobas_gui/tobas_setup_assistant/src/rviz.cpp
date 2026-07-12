// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/rviz.hpp"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_ros2_tools/parameter.hpp>
#include <tobas_urdf/exporter.hpp>
#include <tobas_xml_tools/core.hpp>

#include "tobas_setup_assistant/constants.hpp"
#include "tobas_setup_assistant/util.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
RvizWidget::RvizWidget(const uadf::Model& uadf, const kdl::Tree& tree)
  : uadf_(uadf), tree_(tree), rviz_manager_("rviz_robot_state_display")
{
  // Declare ROS parameters.
  constexpr char kMinimulUrdf[] = "<robot name=\"empty\"><link name=\"root\"/></robot>";
  ros2::declareParam(rviz_manager_.rawNode(), kRobotDescriptionParam, kMinimulUrdf);

  // Initialize RViz.
  const auto rviz_config_path = getPkgShareDir() / "config/setup_assistant.rviz";
  rviz_manager_.initialize(QString::fromStdString(rviz_config_path));
  resetOrbitView();

  // Set up robot_model_display
  display_ = rviz_manager_.getDisplays("RobotState").at(0);

  // Get properties to use.
  enable_visual_ = qt::qPointerCast<rviz_common::properties::BoolProperty>(display_->subProp("Visual Enabled"));
  enable_collision_ = qt::qPointerCast<rviz_common::properties::BoolProperty>(display_->subProp("Collision Enabled"));
  enable_inertia_ = qt::qPointerCast<rviz_common::properties::BoolProperty>(display_->subProp("Inertial Enabled"));
  highlight_link_ = qt::qPointerCast<rviz_common::properties::StringProperty>(display_->subProp("Highlight Link"));
  unhighlight_link_ = qt::qPointerCast<rviz_common::properties::StringProperty>(display_->subProp("Unhighlight Link"));
  reload_ = qt::qPointerCast<rviz_common::properties::BoolProperty>(display_->subProp("Reload"));

  enable_visual_->setBool(kDefaultVisualEnabled);
  enable_collision_->setBool(kDefaultCollisionEnabled);
  enable_inertia_->setBool(kDefaultInertiaEnabled);

  // Visualization button.
  const auto visual_box = new QCheckBox("Show Visual");
  visual_box->setChecked(kDefaultVisualEnabled);
  const auto collision_box = new QCheckBox("Show Collision");
  collision_box->setChecked(kDefaultCollisionEnabled);
  const auto inertia_box = new QCheckBox("Show Inertial");
  inertia_box->setChecked(kDefaultInertiaEnabled);

  // Layout.
  const auto rows = new QVBoxLayout();
  setLayout(rows);
  const auto cols = new QHBoxLayout();
  rows->addWidget(rviz_manager_.widget());
  rows->addLayout(cols);
  cols->addStretch();
  cols->addWidget(visual_box);
  cols->addWidget(collision_box);
  cols->addWidget(inertia_box);

  // Connection
  connect(visual_box, &QCheckBox::toggled, this, &self::onVisualBoxToggled);
  connect(collision_box, &QCheckBox::toggled, this, &self::onCollisionBoxToggled);
  connect(inertia_box, &QCheckBox::toggled, this, &self::onInertiaBoxToggled);
}

void RvizWidget::updateInternalDataStructures()
{
  // Reset the orbit view
  resetOrbitView();

  // Set the fixed frame to the root link.
  const auto& root_name = tree_.getRootName();
  rviz_manager_.setFixedFrame(QString::fromStdString(root_name));

  // Update the URDF.
  const auto urdf_doc = urdf::exportUrdf(*uadf_.urdf);
  const auto urdf_text = xml::xmlDocumentToString(urdf_doc);
  rviz_manager_.rawNode()->set_parameter(rclcpp::Parameter(kRobotDescriptionParam, urdf_text));

  // Reload the robot model.
  reload_->setBool(false);
  reload_->setBool(true);
}

void RvizWidget::heightLink(const QString& link_name)
{
  if (link_name == highlighted_link_) {
    return;
  }

  if (!highlighted_link_.isEmpty()) {
    unheightLink(highlighted_link_);
  }

  highlight_link_->setValue(link_name);
  highlighted_link_ = link_name;
}

void RvizWidget::unheightLink(const QString& link_name)
{
  unhighlight_link_->setValue(link_name);
}

void RvizWidget::resetOrbitView()
{
  rviz_manager_.setOrbitView(1.7f, M_PIf, M_PI_4f, 0.0f, 0.0f, 0.0f);
}

void RvizWidget::onVisualBoxToggled(bool checked)
{
  enable_visual_->setBool(checked);
}

void RvizWidget::onCollisionBoxToggled(bool checked)
{
  enable_collision_->setBool(checked);
}

void RvizWidget::onInertiaBoxToggled(bool checked)
{
  enable_inertia_->setBool(checked);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
