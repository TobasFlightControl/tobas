// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_wrapper/rviz.hpp"

#include <OgreMaterialManager.h>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_frame.hpp>  // `rviz_rendering/render_window.hpp` does not support `DQT_NO_KEYWORD`.
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/yaml_config_reader.hpp>

namespace tobas
{
namespace rviz
{
RvizFrameManager::RvizFrameManager(int argc, char** argv, const std::string& node_name)
{
  // Initialize ROS node.
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  // Create the RViz ROS interface.
  node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name);
}

RvizFrameManager::RvizFrameManager(const std::string& node_name) : RvizFrameManager(0, nullptr, node_name)
{
}

RvizFrameManager::~RvizFrameManager()
{
  clear();
}

void RvizFrameManager::initialize(const QString& config_path)
{
  removeDefaultColorMaterials();

  // Read configuration.
  rviz_common::YamlConfigReader reader;
  rviz_common::Config config;
  reader.readFile(config, config_path);

  // Create a visualization frame.
  // Keep the frame parentless so that QLayout::addWidget() reparents it as an embedded widget and clears its Qt::Window flag.
  frame_ = new rviz_common::VisualizationFrame(node_, nullptr);

  // Initialize the visualization frame.
  frame_->setSplashPath("");  // Do not show a splash image.
  frame_->initialize(node_);  // The initialization method must be called after the splash path is set.

  // Configure the visualization frame.
  frame_->load(config);
  frame_->setMenuBar(nullptr);
  frame_->setStatusBar(nullptr);
  frame_->setHideButtonVisibility(false);
  frame_->setStyleSheet("QSizeGrip { width: 0px; height: 0px; }");  // Remove sizegrip.

  // Get child instances.
  manager_ = frame_->getManager();
  display_group_ = manager_->getRootDisplayGroup();
}

void RvizFrameManager::clear()
{
  if (manager_) {
    manager_->stopUpdate();
    manager_->removeAllDisplays();
  }

  delete frame_;
  frame_ = nullptr;
  manager_ = nullptr;
  display_group_ = nullptr;

  node_.reset();
}

rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr RvizFrameManager::rvizNode()
{
  return node_;
}

rclcpp::Node::SharedPtr RvizFrameManager::rawNode()
{
  return node_->get_raw_node();
}

QWidget* RvizFrameManager::widget()
{
  return frame_;
}

void RvizFrameManager::resetTime()
{
  if (manager_) {
    manager_->resetTime();
  }
}

QString RvizFrameManager::getFixedFrame() const
{
  return manager_->getFixedFrame();
}

void RvizFrameManager::setFixedFrame(const QString& frame)
{
  manager_->setFixedFrame(frame);
}

void RvizFrameManager::setOrbitView(
  const float distance,
  const float yaw,
  const float pitch,
  const float focal_x,
  const float focal_y,
  const float focal_z)
{
  const auto view_manager = manager_->getViewManager();
  view_manager->setCurrentViewControllerType("rviz_default_plugins/Orbit");
  const auto view = view_manager->getCurrent();
  if (!view) {
    RCLCPP_WARN(rawNode()->get_logger(), "Failed to get the current RViz view controller.");
    return;
  }

  const auto set_float = [this, view](const char* name, const float value)
  {
    const auto prop = qobject_cast<rviz_common::properties::FloatProperty*>(view->subProp(name));
    if (!prop) {
      RCLCPP_WARN_STREAM(rawNode()->get_logger(), "Failed to get RViz view property: " << name);
      return;
    }
    prop->setFloat(value);
  };

  set_float("Distance", distance);
  set_float("Yaw", yaw);
  set_float("Pitch", pitch);

  const auto focal_point = qobject_cast<rviz_common::properties::VectorProperty*>(view->subProp("Focal Point"));
  if (!focal_point) {
    RCLCPP_WARN(rawNode()->get_logger(), "Failed to get RViz view property: Focal Point");
    return;
  }
  focal_point->setVector(Ogre::Vector3(focal_x, focal_y, focal_z));
}

std::vector<rviz_common::Display*> RvizFrameManager::getDisplays(const QString& name)
{
  std::vector<rviz_common::Display*> res;

  for (int i = 0; i < display_group_->numDisplays(); ++i) {
    const auto display = display_group_->getDisplayAt(i);

    if (!display) {
      RCLCPP_WARN_STREAM(rawNode()->get_logger(), "Failed to get display of index " << std::to_string(i));
      continue;
    };

    if (display->getName() == name) {
      res.push_back(display);
    }
  }

  return res;
}

void RvizFrameManager::removeDefaultColorMaterials()
{
  const auto material_manager = Ogre::MaterialManager::getSingletonPtr();

  if (!material_manager) {
    return;
  }

  // Delete this to prevent duplication of materials created by `rviz_rendering::MaterialManager::createDefaultColorMaterials()`.
  // TODO: Remove this process when it is fixed upstream.
  material_manager->remove("RVIZ/Red", "rviz_rendering");
  material_manager->remove("RVIZ/Green", "rviz_rendering");
  material_manager->remove("RVIZ/Blue", "rviz_rendering");
  material_manager->remove("RVIZ/Cyan", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedRed", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedGreen", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedBlue", "rviz_rendering");
  material_manager->remove("RVIZ/ShadedCyan", "rviz_rendering");
}
}  // namespace rviz
}  // namespace tobas
