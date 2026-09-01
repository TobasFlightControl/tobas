// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/ogre_helpers/ogre_controller.hpp"

#include <optional>

#include <OgreSceneManager.h>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include "tobas_urdf_builder_plugin/ogre_helpers/static_link_updater.hpp"
#include "tobas_urdf_builder_plugin/utils/constants.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ogre
{
namespace
{
constexpr float kAxesLength = 0.1;
constexpr float kAxesRadius = 0.01;
constexpr float kCharHeight = 0.03;  // Character size displayed in the model view.

constexpr float kHighlightR = 0.0;
constexpr float kHighlightG = 1.0;
constexpr float kHighlightB = 0.0;
}  // namespace

struct OgreController::PImpl
{
  explicit PImpl(rviz_common::DisplayContext* context) : ogre(context->getSceneManager())
  {
    ogre.root_node = ogre.scene_manager->getRootSceneNode();
    ogre.robot_node = ogre.root_node->createChildSceneNode();
    ogre.axes_node = ogre.root_node->createChildSceneNode();
    ogre.names_node = ogre.root_node->createChildSceneNode();
    rviz.robot = new rviz_default_plugins::robot::Robot(ogre.robot_node, context, "urdf_robot_model", nullptr);

    rviz.robot->setAlpha(kDefaultRobotAlpha);  // Alpha for the entire robot.
    rviz.robot->setVisualVisible(kDefaultVisualVisible);
    rviz.robot->setCollisionVisible(kDefaultCollisionVisible);
  }

  ~PImpl()
  {
    // FIXME: An error occurs when freeing OGRE memory. In practice, the process exits when `PImpl` is released,
    // so leaking memory is acceptable. ogre.root_node->removeAndDestroyAllChildren();
    // ogre.scene_manager->destroySceneNode(ogre.root_node->getName());
  }

  struct RvizPrivate_
  {
    rviz_default_plugins::robot::Robot* robot;
    std::vector<std::shared_ptr<rviz_rendering::Axes>> axes;
  } rviz;

  struct OgrePrivate_
  {
    explicit OgrePrivate_(Ogre::SceneManager* _scene_manager) : scene_manager(_scene_manager)
    {
    }

    Ogre::SceneManager* scene_manager;
    Ogre::SceneNode* root_node = nullptr;
    Ogre::SceneNode* robot_node = nullptr;
    Ogre::SceneNode* axes_node = nullptr;
    Ogre::SceneNode* names_node = nullptr;
  } ogre;

  std::optional<ogre::StaticLinkUpdater> link_updater;
};

OgreController::OgreController(rviz_common::DisplayContext* context) : pimpl_(new OgreController::PImpl(context))
{
}

OgreController::~OgreController() = default;

void OgreController::update()
{
  if (!pimpl_->rviz.robot || !pimpl_->link_updater) {
    return;
  }

  pimpl_->rviz.robot->update(*pimpl_->link_updater);
}

void OgreController::reload(const view_model::URDFViewModel& vm)
{
  reloadRobot(vm);
  reloadAxes(vm);
}

void OgreController::reloadRobot(const view_model::URDFViewModel& vm)
{
  const auto& model = vm.urdf();
  pimpl_->rviz.robot->load(*model);
  pimpl_->link_updater.emplace(model);

  for (const auto& [name, link] : pimpl_->rviz.robot->getLinks()) {
    if (highlighted_links_.find(name) != highlighted_links_.end()) {
      link->setColor(kHighlightR, kHighlightG, kHighlightB);
    }
    else {
      link->unsetColor();
    }

    if (hidden_links_.find(name) != hidden_links_.end()) {
      link->setRobotAlpha(0.0);
    }
    else {
      link->setRobotAlpha(kDefaultRobotAlpha);
    }
  }
}

void OgreController::reloadAxes(const view_model::URDFViewModel& vm)
{
  pimpl_->ogre.names_node->removeAndDestroyAllChildren();
  pimpl_->rviz.axes.clear();

  const auto& model = vm.urdf();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  for (const auto& pair : model->links_) {
    if (!pimpl_->link_updater->getLinkTransforms(pair.second->name, position, orientation, position, orientation)) {
      continue;
    }

    const auto axes = std::make_shared<rviz_rendering::Axes>(
      pimpl_->ogre.scene_manager, pimpl_->ogre.axes_node, kAxesLength, kAxesRadius);
    axes->setPosition(position);
    axes->setOrientation(orientation);
    pimpl_->rviz.axes.push_back(axes);

    auto name_text = new rviz_rendering::MovableText(pair.second->name, "Liberation Sans", kCharHeight);
    name_text->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_BELOW);

    auto name_node = pimpl_->ogre.names_node->createChildSceneNode();
    name_node->setPosition(position);
    name_node->attachObject(name_text);
  }
}

void OgreController::highlight(const std::string& link_name)
{
  const auto link = pimpl_->rviz.robot->getLink(link_name);
  link->setColor(kHighlightR, kHighlightG, kHighlightB);
  highlighted_links_.insert(link_name);
}

void OgreController::unhighlight(const std::string& link_name)
{
  const auto link = pimpl_->rviz.robot->getLink(link_name);
  link->unsetColor();
  highlighted_links_.erase(link_name);
}

void OgreController::unhighlightAll()
{
  for (const auto& pair : pimpl_->rviz.robot->getLinks()) {
    pair.second->unsetColor();
  }
  highlighted_links_.clear();
}

void OgreController::show(const std::string& link_name)
{
  const auto link = pimpl_->rviz.robot->getLink(link_name);
  link->setRobotAlpha(kDefaultRobotAlpha);
  hidden_links_.erase(link_name);
}

void OgreController::hide(const std::string& link_name)
{
  const auto link = pimpl_->rviz.robot->getLink(link_name);
  link->setRobotAlpha(0.0);
  hidden_links_.insert(link_name);
}

void OgreController::setVisualVisible(bool visible)
{
  pimpl_->rviz.robot->setVisualVisible(visible);
}

void OgreController::setCollisionVisible(bool visible)
{
  pimpl_->rviz.robot->setCollisionVisible(visible);
}

void OgreController::setInertiaVisible(bool visible)
{
  pimpl_->rviz.robot->setInertiaVisible(visible);
}
}  // namespace ogre
}  // namespace ub
}  // namespace gui
}  // namespace tobas
