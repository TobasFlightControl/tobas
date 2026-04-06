// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_state_display.hpp"

#include <rviz_common/properties/string_property.hpp>

#include "tobas_rviz_plugin/conversions.hpp"
#include "tobas_rviz_plugin/link_updater.hpp"

namespace tobas
{
RobotStateDisplay::RobotStateDisplay()
{
  robot_description_property_ = new rviz_common::properties::StringProperty(
    "Robot Description",
    "robot_description",
    "The name of the ROS parameter where the URDF for the robot is loaded",
    this,
    SLOT(changedRobotDescription()),
    this);

  robot_state_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Robot State Topic",
    "display_robot_state",
    rosidl_generator_traits::data_type<tobas_visualization_msgs::msg::DisplayRobotState>(),
    "The topic on which the tobas_visualization_msgs::msg::DisplayRobotState messages are received",
    this,
    SLOT(changedRobotStateTopic()),
    this);

  root_link_name_property_ = new rviz_common::properties::StringProperty(
    "Robot Root Link",
    "",
    "Shows the name of the root link for the robot model",
    this,
    SLOT(changedRootLinkName()),
    this);
  root_link_name_property_->setReadOnly(true);

  robot_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Robot Alpha", 1.0f, "Specifies the alpha for the robot links", this, SLOT(changedRobotSceneAlpha()), this);
  robot_alpha_property_->setMin(0.);
  robot_alpha_property_->setMax(1.);

  enable_link_highlight_ = new rviz_common::properties::BoolProperty(
    "Show Highlights",
    true,
    "Specifies whether link highlighting is enabled",
    this,
    SLOT(changedEnableLinkHighlight()),
    this);
  enable_visual_visible_ = new rviz_common::properties::BoolProperty(
    "Visual Enabled",
    true,
    "Whether to display the visual representation of the robot.",
    this,
    SLOT(changedEnableVisualVisible()),
    this);
  enable_collision_visible_ = new rviz_common::properties::BoolProperty(
    "Collision Enabled",
    false,
    "Whether to display the collision representation of the robot.",
    this,
    SLOT(changedEnableCollisionVisible()),
    this);

  enable_inertia_visible_ = new rviz_common::properties::BoolProperty(
    "Inertial Enabled",
    false,
    "Whether to display the inertia representation of the robot.",
    this,
    SLOT(changedEnableInertiaVisible()),
    this);

  show_all_links_ = new rviz_common::properties::BoolProperty(
    "Show All Links", true, "Toggle all links visibility on or off.", this, SLOT(changedAllLinks()), this);

  highlight_link_ = new rviz_common::properties::StringProperty(
    "Highlight Link", "", "Highlight chosen link.", this, SLOT(changedHighlightColor()), this);

  unhighlight_link_ = new rviz_common::properties::StringProperty(
    "Unhighlight Link", "", "Unhighlight chosen link.", this, SLOT(changedUnhighlightColor()), this);

  reload_ =
    new rviz_common::properties::BoolProperty("Reload", true, "Reload robot model.", this, SLOT(changedReload()), this);
}

void RobotStateDisplay::load(const rviz_common::Config& config)
{
  // This property needs to be loaded in onEnable() below, which is triggered
  // in the beginning of super::load() before the other property would be available
  robot_description_property_->load(config.mapGetChild("Robot Description"));
  super::load(config);
}

void RobotStateDisplay::update(float wall_dt, float ros_dt)
{
  super::update(wall_dt, ros_dt);
  calculateOffsetPosition();
  if (robot_ && update_state_ && robot_state_) {
    update_state_ = false;
    robot_state_->update();
    robot_->update(LinkUpdater(robot_state_));
  }
}

void RobotStateDisplay::reset()
{
  robot_->clear();
  rdf_loader_.reset();
  super::reset();
  if (isEnabled()) {
    onEnable();
  }
}

void RobotStateDisplay::setLinkColor(const std::string& link_name, const QColor& color)
{
  const auto link = robot_->getLink(link_name);
  if (link) {
    link->setColor(color.redF(), color.greenF(), color.blueF());
  }
}

void RobotStateDisplay::unsetLinkColor(const std::string& link_name)
{
  const auto link = robot_->getLink(link_name);
  if (link) {
    link->unsetColor();
  }
}

void RobotStateDisplay::setVisible(bool visible)
{
  robot_->setVisible(visible);
}

void RobotStateDisplay::onInitialize()
{
  super::onInitialize();

  const auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    RVIZ_COMMON_LOG_WARNING("Unable to lock weak_ptr from DisplayContext in RobotStateDisplay constructor");
    return;
  }

  robot_state_topic_property_->initialize(ros_node_abstraction);
  node_ = ros_node_abstraction->get_raw_node();
  robot_ = std::make_shared<rviz_default_plugins::robot::Robot>(scene_node_, context_, "Robot State", this);

  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  changedEnableInertiaVisible();
  robot_->setVisible(false);
}

void RobotStateDisplay::onEnable()
{
  super::onEnable();
  if (!rdf_loader_) {
    initializeLoader();
  }
  changedRobotStateTopic();
  calculateOffsetPosition();
}

void RobotStateDisplay::onDisable()
{
  robot_state_sub_.reset();
  if (robot_) {
    robot_->setVisible(false);
  }
  super::onDisable();
}

void RobotStateDisplay::fixedFrameChanged()
{
  super::fixedFrameChanged();
  calculateOffsetPosition();
}

void RobotStateDisplay::initializeLoader()
{
  if (robot_description_property_->getStdString().empty()) {
    setStatus(rviz_common::properties::StatusProperty::Error, "RobotModel", "`Robot Description` field can't be empty");
    return;
  }

  rdf_loader_ = std::make_shared<RDFLoader>(node_, robot_description_property_->getStdString(), true);
  loadRobotModel();
  rdf_loader_->setNewModelCallback([this]() { return loadRobotModel(); });
}

void RobotStateDisplay::loadRobotModel()
{
  if (rdf_loader_->getURDF()) {
    try {
      const auto srdf = std::make_shared<srdf::Model>();
      robot_model_ = std::make_shared<RobotModel>(rdf_loader_->getURDF(), srdf);

      robot_->load(*robot_model_->getURDF());
      robot_->setVisualVisible(enable_visual_visible_->getBool());
      robot_->setCollisionVisible(enable_collision_visible_->getBool());
      robot_->setInertiaVisible(enable_inertia_visible_->getBool());
      robot_->setVisible(true);

      robot_state_ = std::make_shared<RobotState>(robot_model_);
      robot_state_->setToDefaultValues();

      const bool old_state = root_link_name_property_->blockSignals(true);
      root_link_name_property_->setStdString(robot_model_->getRootLinkName());
      root_link_name_property_->blockSignals(old_state);
      update_state_ = true;

      setStatus(rviz_common::properties::StatusProperty::Ok, "RobotModel", "Loaded successfully");
    }
    catch (const std::exception& e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "RobotModel", QString("Loading failed: %1").arg(e.what()));
    }
  }
  else {
    setStatus(rviz_common::properties::StatusProperty::Error, "RobotModel", "Loading failed");
  }

  highlights_.clear();
}

void RobotStateDisplay::calculateOffsetPosition()
{
  if (!robot_model_) {
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(
    robot_model_->getModelFrame(), rclcpp::Time(0, 0, RCL_ROS_TIME), position, orientation);

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void RobotStateDisplay::newRobotStateCallback(
  const tobas_visualization_msgs::msg::DisplayRobotState::ConstSharedPtr& state_msg)
{
  if (!robot_model_) {
    return;
  }
  if (!robot_state_) {
    robot_state_ = std::make_shared<RobotState>(robot_model_);
  }
  // possibly use TF to construct a Transforms object to pass in to the conversion function?
  try {
    robotStateMsgToRobotState(state_msg->state, *robot_state_);
    setRobotHighlights(state_msg->highlight_links);
  }
  catch (const Exception& e) {
    robot_state_->setToDefaultValues();
    setRobotHighlights(tobas_visualization_msgs::msg::DisplayRobotState::_highlight_links_type());
    setStatus(rviz_common::properties::StatusProperty::Error, "RobotState", e.what());
    robot_->setVisible(false);
    return;
  }

  if (robot_->isVisible() != !state_msg->hide) {
    robot_->setVisible(!state_msg->hide);
    if (robot_->isVisible()) {
      setStatus(rviz_common::properties::StatusProperty::Ok, "RobotState", "");
    }
    else {
      setStatus(rviz_common::properties::StatusProperty::Warn, "RobotState", "Hidden");
    }
  }

  update_state_ = true;
}

void RobotStateDisplay::setRobotHighlights(
  const tobas_visualization_msgs::msg::DisplayRobotState::_highlight_links_type& links)
{
  if (links.empty() && highlights_.empty()) {
    return;
  }

  std::map<std::string, std_msgs::msg::ColorRGBA> highlights;
  for (const auto& link : links) {
    highlights[link.id] = link.color;
  }

  if (enable_link_highlight_->getBool()) {
    auto ho = highlights_.begin();
    auto hn = highlights.begin();
    while (ho != highlights_.end() || hn != highlights.end()) {
      if (ho == highlights_.end()) {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn == highlights.end()) {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->first < ho->first) {
        setHighlight(hn->first, hn->second);
        ++hn;
      }
      else if (hn->first > ho->first) {
        unsetHighlight(ho->first);
        ++ho;
      }
      else if (hn->second != ho->second) {
        setHighlight(hn->first, hn->second);
        ++ho;
        ++hn;
      }
      else {
        ++ho;
        ++hn;
      }
    }
  }

  swap(highlights, highlights_);
}

void RobotStateDisplay::setHighlight(const std::string& link_name, const std_msgs::msg::ColorRGBA& color)
{
  const auto link = robot_->getLink(link_name);
  if (link) {
    link->setColor(color.r, color.g, color.b);
    link->setRobotAlpha(color.a * robot_alpha_property_->getFloat());
  }
}

void RobotStateDisplay::unsetHighlight(const std::string& link_name)
{
  const auto link = robot_->getLink(link_name);
  if (link) {
    link->unsetColor();
    link->setRobotAlpha(robot_alpha_property_->getFloat());
  }
}

void RobotStateDisplay::changedRobotDescription()
{
  if (isEnabled()) {
    reset();
  }
}

void RobotStateDisplay::changedRootLinkName()
{
}

void RobotStateDisplay::changedRobotSceneAlpha()
{
  if (robot_) {
    robot_->setAlpha(robot_alpha_property_->getFloat());
    update_state_ = true;
  }
}

void RobotStateDisplay::changedRobotStateTopic()
{
  // Reset model to default state, we don't want to show previous messages
  if (static_cast<bool>(robot_state_)) {
    robot_state_->setToDefaultValues();
  }
  update_state_ = true;
  robot_->setVisible(false);
  setStatus(rviz_common::properties::StatusProperty::Warn, "RobotState", "No msg received");

  robot_state_sub_ = node_->create_subscription<tobas_visualization_msgs::msg::DisplayRobotState>(
    robot_state_topic_property_->getStdString(),
    rclcpp::SystemDefaultsQoS(),
    [this](const tobas_visualization_msgs::msg::DisplayRobotState::ConstSharedPtr& state)
    { return newRobotStateCallback(state); });
}

void RobotStateDisplay::changedEnableLinkHighlight()
{
  if (enable_link_highlight_->getBool()) {
    for (std::pair<const std::string, std_msgs::msg::ColorRGBA>& highlight : highlights_) {
      setHighlight(highlight.first, highlight.second);
    }
  }
  else {
    for (std::pair<const std::string, std_msgs::msg::ColorRGBA>& highlight : highlights_) {
      unsetHighlight(highlight.first);
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

void RobotStateDisplay::changedEnableInertiaVisible()
{
  robot_->setInertiaVisible(enable_inertia_visible_->getBool());
}

void RobotStateDisplay::changedAllLinks()
{
  const auto links_prop = subProp("Links");
  const QVariant value(show_all_links_->getBool());

  for (int i = 0; i < links_prop->numChildren(); ++i) {
    const auto link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

void RobotStateDisplay::changedHighlightColor()
{
  if (!robot_) {
    RCLCPP_ERROR(node_->get_logger(), "Robot is null.");
    return;
  }

  std_msgs::msg::ColorRGBA color_msg;
  color_msg.r = 0.0f;
  color_msg.g = 1.0f;
  color_msg.b = 0.0f;
  color_msg.a = 0.7f;
  setHighlight(highlight_link_->getStdString(), color_msg);
  update_state_ = true;
}

void RobotStateDisplay::changedUnhighlightColor()
{
  if (!robot_) {
    RCLCPP_ERROR(node_->get_logger(), "Robot is null.");
    return;
  }

  unsetHighlight(unhighlight_link_->getStdString());
  update_state_ = true;
}

void RobotStateDisplay::changedReload()
{
  if (reload_->getBool()) {
    reset();
  }
}
}  // namespace tobas
