// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_ros/visualization/robot_state_rviz_plugin/include/moveit/robot_state_rviz_plugin/robot_state_display.hpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#pragma once

#include <chrono>
#include <optional>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

#include <tobas_visualization_msgs/msg/display_robot_state.hpp>

#include "./rdf_loader.hpp"
#include "./robot_model.hpp"
#include "./robot_state.hpp"

namespace tobas::rviz
{
class RobotStateDisplay : public rviz_common::Display
{
  Q_OBJECT

  using super = rviz_common::Display;

public:
  explicit RobotStateDisplay();

  void load(const rviz_common::Config& _config) override;
  void update(std::chrono::nanoseconds _wall_dt, std::chrono::nanoseconds _ros_dt) override;
  void reset() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

  void initializeLoader();
  void loadRobotModel();

  void calculateOffsetPosition();

  void newRobotStateCallback(const tobas_visualization_msgs::msg::DisplayRobotState::ConstSharedPtr& _state);

  void setRobotHighlights(const tobas_visualization_msgs::msg::DisplayRobotState::_highlight_links_type& _links);
  void setHighlight(const std::string& _link_name, const std_msgs::msg::ColorRGBA& _color);
  void unsetHighlight(const std::string& _link_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<tobas_visualization_msgs::msg::DisplayRobotState>::SharedPtr robot_state_sub_;

  std::optional<RDFLoader> rdf_loader_;
  std::optional<rviz_default_plugins::robot::Robot> robot_;
  std::optional<RobotModel> robot_model_;
  std::optional<RobotState> robot_state_;
  std::map<std::string, std_msgs::msg::ColorRGBA> highlights_;
  bool update_state_ = false;

  rviz_common::properties::StringProperty* robot_description_property_;
  rviz_common::properties::StringProperty* root_link_name_property_;
  rviz_common::properties::RosTopicProperty* robot_state_topic_property_;
  rviz_common::properties::FloatProperty* robot_alpha_property_;
  rviz_common::properties::BoolProperty* enable_link_highlight_;
  rviz_common::properties::BoolProperty* enable_visual_visible_;
  rviz_common::properties::BoolProperty* enable_collision_visible_;
  rviz_common::properties::BoolProperty* enable_inertia_visible_;
  rviz_common::properties::BoolProperty* show_all_links_;
  rviz_common::properties::StringProperty* highlight_link_;
  rviz_common::properties::StringProperty* unhighlight_link_;
  rviz_common::properties::BoolProperty* reload_;

private Q_SLOTS:
  void changedRobotDescription();
  void changedRootLinkName();
  void changedRobotSceneAlpha();
  void changedRobotStateTopic();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedEnableInertiaVisible();
  void changedAllLinks();
  void changedHighlightColor();
  void changedUnhighlightColor();
  void changedReload();
};
}  // namespace tobas::rviz
