#include <OGRE/OgreSceneManager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/visualization_manager.h>

#include "../../include/urdf_builder/ogre_helpers/static_link_updater.hpp"
#include "../../include/urdf_builder/ogre_helpers/ogre_controller.hpp"
#include "../../include/urdf_builder/ui/display_context_proxy.hpp"
#include "../../include/urdf_builder/utils/constants.hpp"

using namespace std;

namespace urdf_builder
{
namespace ogre_helpers
{
struct OgreController::PImpl
{
  explicit PImpl(rviz::VisualizationManager* visualization_manager)
    : rviz(visualization_manager),
      ogre(visualization_manager->getSceneManager()),
      link_updater(nullptr)
  {
    ogre.root_node = ogre.scene_manager->getRootSceneNode();
    ogre.robot_node = ogre.root_node->createChildSceneNode();
    ogre.axes_node = ogre.root_node->createChildSceneNode();
    ogre.names_node = ogre.root_node->createChildSceneNode();
    display_context_proxy =
      new ui::DisplayContextProxy(ogre.scene_manager, visualization_manager->getSelectionManager());
    rviz.robot.reset(
      new rviz::Robot(ogre.robot_node, display_context_proxy, "urdf_robot_model", nullptr));

    rviz.robot->setAlpha(kDefaultRobotAlpha);  // ロボット全体のAlpha
    rviz.robot->setVisualVisible(kDefaultVisualVisible);
    rviz.robot->setCollisionVisible(kDefaultCollisionVisible);
  }

  ~PImpl()
  {
    delete display_context_proxy;

    // FIXME: ogreのメモリ解放時にエラーが出る (実用上PImplのメモリはリークしても問題ないかも)
    // ogre.root_node->removeAndDestroyAllChildren();
    // ogre.scene_manager->destroySceneNode(ogre.root_node->getName());
  }

  struct RvizPrivate_
  {
    explicit RvizPrivate_(rviz::VisualizationManager* _visualization_manager)
      : visualization_manager(_visualization_manager)
    {
    }

    rviz::VisualizationManager* visualization_manager;
    rviz::RobotPtr robot;
    vector<rviz::AxesPtr> axes;
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

  ogre_helpers::StaticLinkUpdaterPtr link_updater;
  ui::DisplayContextProxy* display_context_proxy;
};

OgreController::OgreController(rviz::VisualizationManager* visualization_manager)
  : pimpl_(new OgreController::PImpl(visualization_manager))
{
}

OgreController::~OgreController() = default;

void OgreController::update()
{
  if (pimpl_->rviz.robot == nullptr || pimpl_->link_updater == nullptr)
    return;

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
  pimpl_->link_updater.reset(new ogre_helpers::StaticLinkUpdater(model));

  for (const auto& pair : pimpl_->rviz.robot->getLinks())
  {
    const auto& name = pair.first;
    const auto& link = pair.second;

    if (highlighted_links_.contains(name))
      link->setColor(kHighlightR, kHighlightG, kHighlightB);
    else
      link->unsetColor();

    if (hidden_links_.contains(name))
      link->setRobotAlpha(0.);
    else
      link->setRobotAlpha(kDefaultRobotAlpha);
  }
}

void OgreController::reloadAxes(const view_model::URDFViewModel& vm)
{
  pimpl_->ogre.names_node->removeAndDestroyAllChildren();
  pimpl_->rviz.axes.clear();

  const auto& model = vm.urdf();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  for (const auto& pair : model->links_)
  {
    if (!pimpl_->link_updater->getLinkTransforms(
          pair.second->name, position, orientation, position, orientation))
      continue;

    rviz::AxesPtr axes(
      new rviz::Axes(pimpl_->ogre.scene_manager, pimpl_->ogre.axes_node, kAxesLength, kAxesRadius));
    axes->setPosition(position);
    axes->setOrientation(orientation);
    pimpl_->rviz.axes.push_back(axes);

    auto name_text = new rviz::MovableText(pair.second->name, "Liberation Sans", kCharHeight);
    name_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);

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
  for (const auto& pair : pimpl_->rviz.robot->getLinks())
    pair.second->unsetColor();
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
  link->setRobotAlpha(0.);
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
}  // namespace ogre_helpers
}  // namespace urdf_builder
