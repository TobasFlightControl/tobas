#include <moveit/robot_state/conversions.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "../../include/robot_state_rviz_plugin/robot_state_display.hpp"

using namespace std;

namespace moveit_rviz_plugin
{
RobotStateDisplay::RobotStateDisplay() : Display(), update_state_(false), load_robot_model_(false)
{
  robot_description_property_ = new rviz::StringProperty(
    "Robot Description", "robot_description",
    "The name of the ROS parameter where the URDF for the robot is loaded", this,
    SLOT(changedRobotDescription()), this);

  robot_state_topic_property_ = new rviz::RosTopicProperty(
    "Robot State Topic", "display_robot_state",
    ros::message_traits::datatype<moveit_msgs::DisplayRobotState>(),
    "The topic on which the moveit_msgs::RobotState messages are received", this,
    SLOT(changedRobotStateTopic()), this);

  // Planning scene category
  // -------------------------------------------------------------------------------------------
  root_link_name_property_ = new rviz::StringProperty(
    "Robot Root Link", "", "Shows the name of the root link for the robot model", this,
    SLOT(changedRootLinkName()), this);
  root_link_name_property_->setReadOnly(true);

  robot_alpha_property_ = new rviz::FloatProperty(
    "Robot Alpha", 1.0f, "Specifies the alpha for the robot links", this,
    SLOT(changedRobotSceneAlpha()), this);
  robot_alpha_property_->setMin(0.0);
  robot_alpha_property_->setMax(1.0);

  attached_body_color_property_ = new rviz::ColorProperty(
    "Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies", this,
    SLOT(changedAttachedBodyColor()), this);

  enable_link_highlight_ = new rviz::BoolProperty(
    "Show Highlights", true, "Specifies whether link highlighting is enabled", this,
    SLOT(changedEnableLinkHighlight()), this);
  enable_visual_visible_ = new rviz::BoolProperty(
    "Visual Enabled", true, "Whether to display the visual representation of the robot.", this,
    SLOT(changedEnableVisualVisible()), this);
  enable_collision_visible_ = new rviz::BoolProperty(
    "Collision Enabled", false, "Whether to display the collision representation of the robot.",
    this, SLOT(changedEnableCollisionVisible()), this);

  show_all_links_ = new rviz::BoolProperty(
    "Show All Links", true, "Toggle all links visibility on or off.", this, SLOT(changedAllLinks()),
    this);

  highlight_link_ = new rviz::StringProperty(
    "Highlight Link", "", "Highlight chosen link", this, SLOT(changedHighlightColor()), this);

  unhighlight_link_ = new rviz::StringProperty(
    "Unhighlight Link", "", "Unhighlight chosen link", this, SLOT(changedUnhighlightColor()), this);
}

void RobotStateDisplay::onInitialize()
{
  Display::onInitialize();
  robot_.reset(new RobotStateVisualization(scene_node_, context_, "Robot State", this));
  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  robot_->setVisible(false);
}

void RobotStateDisplay::reset()
{
  robot_->clear();
  rdf_loader_.reset();
  Display::reset();

  loadRobotModel();
}

void RobotStateDisplay::changedAllLinks()
{
  Property* links_prop = subProp("Links");
  QVariant value(show_all_links_->getBool());

  for (int i = 0; i < links_prop->numChildren(); ++i)
  {
    Property* link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

void RobotStateDisplay::setHighlight(const string& link_name, const std_msgs::ColorRGBA& color)
{
  rviz::RobotLink* link = robot_->getRobot().getLink(link_name);
  if (link)
  {
    link->setColor(color.r, color.g, color.b);
    link->setRobotAlpha(color.a * robot_alpha_property_->getFloat());
  }
}

void RobotStateDisplay::unsetHighlight(const string& link_name)
{
  rviz::RobotLink* link = robot_->getRobot().getLink(link_name);
  if (link)
  {
    link->unsetColor();
    link->setRobotAlpha(robot_alpha_property_->getFloat());
  }
}

void RobotStateDisplay::changedEnableLinkHighlight()
{
  if (enable_link_highlight_->getBool())
  {
    for (map<string, std_msgs::ColorRGBA>::iterator it = highlights_.begin();
         it != highlights_.end(); ++it)
    {
      setHighlight(it->first, it->second);
    }
  }
  else
  {
    for (map<string, std_msgs::ColorRGBA>::iterator it = highlights_.begin();
         it != highlights_.end(); ++it)
    {
      unsetHighlight(it->first);
    }
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

static bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r != b.r || a.g != b.g || a.b != b.b || a.a != b.a;
}

void RobotStateDisplay::setRobotHighlights(
  const moveit_msgs::DisplayRobotState::_highlight_links_type& highlight_links)
{
  if (highlight_links.empty() && highlights_.empty())
    return;

  map<string, std_msgs::ColorRGBA> highlights;
  for (moveit_msgs::DisplayRobotState::_highlight_links_type::const_iterator it =
         highlight_links.begin();
       it != highlight_links.end(); ++it)
  {
    highlights[it->id] = it->color;
  }

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

void RobotStateDisplay::changedHighlightColor()
{
  if (robot_)
  {
    std_msgs::ColorRGBA color_msg;
    color_msg.r = 255;
    color_msg.g = 0;
    color_msg.b = 0;
    color_msg.a = 0.7;
    setHighlight(highlight_link_->getStdString(), color_msg);
    update_state_ = true;
  }
}

void RobotStateDisplay::changedUnhighlightColor()
{
  if (robot_)
  {
    unsetHighlight(unhighlight_link_->getStdString());
    update_state_ = true;
  }
}

void RobotStateDisplay::changedAttachedBodyColor()
{
  if (robot_)
  {
    QColor color = attached_body_color_property_->getColor();
    std_msgs::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = robot_alpha_property_->getFloat();
    robot_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void RobotStateDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void RobotStateDisplay::changedRootLinkName()
{
}

void RobotStateDisplay::changedRobotSceneAlpha()
{
  if (robot_)
  {
    robot_->setAlpha(robot_alpha_property_->getFloat());
    QColor color = attached_body_color_property_->getColor();
    std_msgs::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = robot_alpha_property_->getFloat();
    robot_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void RobotStateDisplay::changedRobotStateTopic()
{
  robot_state_subscriber_.shutdown();

  // reset model to default state, we don't want to show previous messages
  if (static_cast<bool>(kstate_))
    kstate_->setToDefaultValues();
  update_state_ = true;

  robot_state_subscriber_ = root_nh_.subscribe(
    robot_state_topic_property_->getStdString(), 10, &RobotStateDisplay::newRobotStateCallback,
    this);
}

void RobotStateDisplay::newRobotStateCallback(
  const moveit_msgs::DisplayRobotStateConstPtr& state_msg)
{
  if (!kmodel_)
    return;
  if (!kstate_)
    kstate_.reset(new robot_state::RobotState(kmodel_));
  // possibly use TF to construct a robot_state::Transforms object to pass in to the conversion
  // functio?
  robot_state::robotStateMsgToRobotState(state_msg->state, *kstate_);
  setRobotHighlights(state_msg->highlight_links);
  update_state_ = true;
}

void RobotStateDisplay::setLinkColor(const string& link_name, const QColor& color)
{
  setLinkColor(&robot_->getRobot(), link_name, color);
}

void RobotStateDisplay::unsetLinkColor(const string& link_name)
{
  unsetLinkColor(&robot_->getRobot(), link_name);
}

void RobotStateDisplay::setLinkColor(
  rviz::Robot* robot,
  const string& link_name,
  const QColor& color)
{
  rviz::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->setColor(color.redF(), color.greenF(), color.blueF());
}

void RobotStateDisplay::unsetLinkColor(rviz::Robot* robot, const string& link_name)
{
  rviz::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->unsetColor();
}

void RobotStateDisplay::loadRobotModel()
{
  load_robot_model_ = false;
  if (!rdf_loader_)
    rdf_loader_.reset(new rdf_loader::RDFLoader(robot_description_property_->getStdString()));

  if (rdf_loader_->getURDF())
  {
    const srdf::ModelSharedPtr& srdf =
      rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : srdf::ModelSharedPtr(new srdf::Model());
    kmodel_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));
    robot_->load(*kmodel_->getURDF());
    kstate_.reset(new robot_state::RobotState(kmodel_));
    kstate_->setToDefaultValues();
    bool oldState = root_link_name_property_->blockSignals(true);
    root_link_name_property_->setStdString(getRobotModel()->getRootLinkName());
    root_link_name_property_->blockSignals(oldState);
    update_state_ = true;
    setStatus(rviz::StatusProperty::Ok, "RobotState", "Planning Model Loaded Successfully");

    changedEnableVisualVisible();
    changedEnableCollisionVisible();
    robot_->setVisible(true);
  }
  else
    setStatus(rviz::StatusProperty::Error, "RobotState", "No Planning Model Loaded");

  highlights_.clear();
}

void RobotStateDisplay::onEnable()
{
  Display::onEnable();
  load_robot_model_ = true;  // allow loading of robot model in update()
  calculateOffsetPosition();
}

void RobotStateDisplay::onDisable()
{
  robot_state_subscriber_.shutdown();
  if (robot_)
    robot_->setVisible(false);
  Display::onDisable();
}

void RobotStateDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  if (load_robot_model_)
  {
    loadRobotModel();
    changedRobotStateTopic();
  }

  calculateOffsetPosition();
  if (robot_ && update_state_ && kstate_)
  {
    update_state_ = false;
    kstate_->update();
    robot_->update(kstate_);
  }
}

void RobotStateDisplay::calculateOffsetPosition()
{
  if (!getRobotModel())
    return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(
    getRobotModel()->getModelFrame(), ros::Time(0), position, orientation);

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void RobotStateDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();
}
}  // namespace moveit_rviz_plugin
