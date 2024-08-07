#include <moveit/robot_state/conversions.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "../include/robot_state_rviz_plugin/robot_state_display.hpp"

using namespace std;

namespace moveit_rviz_plugin
{
RobotStateDisplay::RobotStateDisplay()
{
  robot_description_property_.reset(new rviz::StringProperty(
    "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded.",
    this, SLOT(changedRobotDescription()), this));

  root_link_name_property_.reset(new rviz::StringProperty(
    "Robot Root Link", "", "Shows the name of the root link for the robot model.", this, SLOT(changedRootLinkName()),
    this));
  root_link_name_property_->setReadOnly(true);

  highlight_link_.reset(new rviz::StringProperty(
    "Highlight Link", "", "Highlight chosen link.", this, SLOT(changedHighlightColor()), this));

  unhighlight_link_.reset(new rviz::StringProperty(
    "Unhighlight Link", "", "Unhighlight chosen link.", this, SLOT(changedUnhighlightColor()), this));

  robot_state_topic_property_.reset(new rviz::RosTopicProperty(
    "Robot State Topic", "display_robot_state", rclcpp::message_traits::datatype<moveit_msgs::DisplayRobotState>(),
    "The topic on which the moveit_msgs::RobotState messages are received.", this, SLOT(changedRobotStateTopic()),
    this));

  robot_alpha_property_.reset(new rviz::FloatProperty(
    "Robot Alpha", 1., "Specifies the alpha for the robot links.", this, SLOT(changedRobotSceneAlpha()), this));
  robot_alpha_property_->setMin(0.);
  robot_alpha_property_->setMax(1.);

  attached_body_color_property_.reset(new rviz::ColorProperty(
    "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies.", this,
    SLOT(changedAttachedBodyColor()), this));

  enable_link_highlight_.reset(new rviz::BoolProperty(
    "Show Highlights", true, "Specifies whether link highlighting is enabled.", this,
    SLOT(changedEnableLinkHighlight()), this));

  enable_visual_visible_.reset(new rviz::BoolProperty(
    "Visual Enabled", true, "Whether to display the visual representation of the robot.", this,
    SLOT(changedEnableVisualVisible()), this));

  enable_collision_visible_.reset(new rviz::BoolProperty(
    "Collision Enabled", false, "Whether to display the collision representation of the robot.", this,
    SLOT(changedEnableCollisionVisible()), this));

  show_all_links_.reset(new rviz::BoolProperty(
    "Show All Links", true, "Toggle all links visibility on or off.", this, SLOT(changedShowAllLinks()), this));

  reload_.reset(new rviz::BoolProperty("Reload", true, "Reload robot model.", this, SLOT(changedReload()), this));
}

void RobotStateDisplay::update(float wall_dt, float ros_dt)
{
  super::update(wall_dt, ros_dt);

  if (load_robot_model_)
  {
    loadRobotModel();
    changedRobotStateTopic();
  }

  calculateOffsetPosition();
  if (update_state_ && robot_ != nullptr && kstate_ != nullptr)
  {
    update_state_ = false;
    kstate_->update();
    robot_->update(kstate_);
  }
}

void RobotStateDisplay::reset()
{
  robot_->clear();
  rdf_loader_.reset();
  restartSubscribers();  // ノードを立ち上げ直した時に接続が切れるため登録しなおす
  super::reset();
  loadRobotModel();
}

void RobotStateDisplay::setLinkColor(const string& link_name, const QColor& color)
{
  setLinkColor(&robot_->getRobot(), link_name, color);
}

void RobotStateDisplay::unsetLinkColor(const string& link_name)
{
  unsetLinkColor(&robot_->getRobot(), link_name);
}

void RobotStateDisplay::loadRobotModel()
{
  load_robot_model_ = false;

  if (rdf_loader_ == nullptr)
    rdf_loader_.reset(new rdf_loader::RDFLoader(robot_description_property_->getStdString()));

  if (rdf_loader_->getURDF())
  {
    const srdf::ModelSharedPtr& srdf =
      rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : srdf::ModelSharedPtr(new srdf::Model());
    kmodel_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));
    robot_->load(*kmodel_->getURDF());
    kstate_.reset(new robot_state::RobotState(kmodel_));
    kstate_->setToDefaultValues();
    const bool old_state = root_link_name_property_->blockSignals(true);
    root_link_name_property_->setStdString(kmodel_->getRootLinkName());
    root_link_name_property_->blockSignals(old_state);
    update_state_ = true;
    setStatus(rviz::StatusProperty::Ok, "RobotState", "Planning Model Loaded Successfully");

    changedEnableVisualVisible();
    changedEnableCollisionVisible();
    robot_->setVisible(true);
  }
  else
  {
    setStatus(rviz::StatusProperty::Error, "RobotState", "No Planning Model Loaded");
  }

  highlights_.clear();
}

void RobotStateDisplay::calculateOffsetPosition()
{
  if (kmodel_ == nullptr)
    return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(kmodel_->getModelFrame(), rclcpp::Time(0), position, orientation);

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void RobotStateDisplay::setLinkColor(rviz::Robot* robot, const string& link_name, const QColor& color)
{
  auto* link = robot->getLink(link_name);
  if (link == nullptr)
    return;

  link->setColor(color.redF(), color.greenF(), color.blueF());
}

void RobotStateDisplay::unsetLinkColor(rviz::Robot* robot, const string& link_name)
{
  auto* link = robot->getLink(link_name);
  if (link == nullptr)
    return;

  link->unsetColor();
}

void RobotStateDisplay::setRobotHighlights(const moveit_msgs::DisplayRobotState::_highlight_links_type& links)
{
  if (links.empty() && highlights_.empty())
    return;

  map<string, std_msgs::ColorRGBA> highlights;
  for (auto it = links.begin(); it != links.end(); ++it)
    highlights[it->id] = it->color;

  if (enable_link_highlight_->getBool())
  {
    map<string, std_msgs::ColorRGBA>::iterator ho = highlights_.begin();
    map<string, std_msgs::ColorRGBA>::iterator hn = highlights.begin();
    while (ho != highlights_.end() || hn != highlights.end())
    {
      if (ho == highlights_.end())
      {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn == highlights.end())
      {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->first < ho->first)
      {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn->first > ho->first)
      {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->second != ho->second)
      {
        setHighlight(hn->first, hn->second);
        ++ho;
        ++hn;
      }
      else
      {
        ++ho;
        ++hn;
      }
    }
  }

  swap(highlights, highlights_);
}

void RobotStateDisplay::setHighlight(const string& link_name, const std_msgs::ColorRGBA& color)
{
  auto* link = robot_->getRobot().getLink(link_name);
  if (link == nullptr)
    return;

  link->setColor(color.r, color.g, color.b);
  link->setRobotAlpha(color.a * robot_alpha_property_->getFloat());
}

void RobotStateDisplay::unsetHighlight(const string& link_name)
{
  auto* link = robot_->getRobot().getLink(link_name);
  if (link == nullptr)
    return;

  link->unsetColor();
  link->setRobotAlpha(robot_alpha_property_->getFloat());
}

void RobotStateDisplay::restartSubscribers()
{
  robot_state_sub_.shutdown();
  robot_state_sub_ = createSubscriber(robot_state_topic_property_->getStdString(), 1, &self::robotStateCb, this);
}

void RobotStateDisplay::robotStateCb(const moveit_msgs::DisplayRobotState::ConstSharedPtr& state_msg)
{
  if (kmodel_ == nullptr)
    return;

  if (kstate_ == nullptr)
    kstate_.reset(new robot_state::RobotState(kmodel_));

  // Use TF to construct a robot_state::Transforms object to pass in to the conversion function?
  robot_state::robotStateMsgToRobotState(state_msg->state, *kstate_);
  setRobotHighlights(state_msg->highlight_links);
  update_state_ = true;
}

void RobotStateDisplay::onInitialize()
{
  super::onInitialize();
  robot_.reset(new RobotStateVisualization(scene_node_, context_, "Robot State", this));
  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  robot_->setVisible(false);
}

void RobotStateDisplay::onEnable()
{
  super::onEnable();
  load_robot_model_ = true;  // allow loading of robot model in update()
  calculateOffsetPosition();
}

void RobotStateDisplay::onDisable()
{
  robot_state_sub_.shutdown();
  if (robot_ != nullptr)
    robot_->setVisible(false);
  super::onDisable();
}

void RobotStateDisplay::fixedFrameChanged()
{
  super::fixedFrameChanged();
  calculateOffsetPosition();
}

void RobotStateDisplay::changedRobotDescription()
{
  reset();
}

void RobotStateDisplay::changedRootLinkName()
{
}

void RobotStateDisplay::changedHighlightColor()
{
  if (robot_ == nullptr)
    return;

  std_msgs::ColorRGBA color_msg;
  color_msg.r = kHighlightR;
  color_msg.g = kHighlightG;
  color_msg.b = kHighlightB;
  color_msg.a = kHighlightA;
  setHighlight(highlight_link_->getStdString(), color_msg);
  update_state_ = true;
}

void RobotStateDisplay::changedUnhighlightColor()
{
  if (robot_ == nullptr)
    return;

  unsetHighlight(unhighlight_link_->getStdString());
  update_state_ = true;
}

void RobotStateDisplay::changedRobotStateTopic()
{
  // Reset model to default state, we don't want to show previous messages
  if (kstate_ != nullptr)
    kstate_->setToDefaultValues();

  restartSubscribers();
  update_state_ = true;
}

void RobotStateDisplay::changedRobotSceneAlpha()
{
  if (robot_ == nullptr)
    return;

  robot_->setAlpha(robot_alpha_property_->getFloat());
  const auto color = attached_body_color_property_->getColor();
  std_msgs::ColorRGBA color_msg;
  color_msg.r = color.redF();
  color_msg.g = color.greenF();
  color_msg.b = color.blueF();
  color_msg.a = robot_alpha_property_->getFloat();
  robot_->setDefaultAttachedObjectColor(color_msg);
  update_state_ = true;
}

void RobotStateDisplay::changedAttachedBodyColor()
{
  if (robot_ == nullptr)
    return;

  const QColor color = attached_body_color_property_->getColor();
  std_msgs::ColorRGBA color_msg;
  color_msg.r = color.redF();
  color_msg.g = color.greenF();
  color_msg.b = color.blueF();
  color_msg.a = robot_alpha_property_->getFloat();
  robot_->setDefaultAttachedObjectColor(color_msg);
  update_state_ = true;
}

void RobotStateDisplay::changedEnableLinkHighlight()
{
  if (enable_link_highlight_->getBool())
  {
    for (auto it = highlights_.begin(); it != highlights_.end(); ++it)
      setHighlight(it->first, it->second);
  }
  else
  {
    for (auto it = highlights_.begin(); it != highlights_.end(); ++it)
      unsetHighlight(it->first);
  }
}

void RobotStateDisplay::changedEnableVisualVisible()
{
  robot_->setVisualVisible(enable_visual_visible_->getBool());
}

void RobotStateDisplay::changedEnableCollisionVisible()
{
  robot_->setCollisionVisible(enable_collision_visible_->getBool());
}

void RobotStateDisplay::changedShowAllLinks()
{
  Property* links_prop = subProp("Links");
  QVariant value(show_all_links_->getBool());

  for (int i = 0; i < links_prop->numChildren(); ++i)
  {
    Property* link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

void RobotStateDisplay::changedReload()
{
  if (reload_->getBool())
    reset();
}
}  // namespace moveit_rviz_plugin
