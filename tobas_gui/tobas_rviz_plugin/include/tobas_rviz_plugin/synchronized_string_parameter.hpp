// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_ros/planning/rdf_loader/include/moveit/rdf_loader/synchronized_string_parameter.hpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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

/* Author: David V. Lu!! */

#pragma once

#include <rclcpp/node.hpp>

#include <std_msgs/msg/string.hpp>

namespace tobas::rviz
{
/**
 * SynchronizedStringParameter is a way to load a string from the ROS environment.
 *
 * First it tries to load the string from a parameter.
 * If that fails, it subscribes to a std_msgs::String topic of the same name to get the value.
 *
 * If the parameter is loaded successfully, you can publish the value as a String msg if the publish_NAME param is true.
 *
 * You can specify how long to wait for a subscribed message with NAME_timeout (double in seconds).
 *
 * By default, the subscription will be killed after the first message is received.
 * If the parameter NAME_continuous is true, then the parent_callback will be called on every subsequent message.
 */
class SynchronizedStringParameter
{
  using StringCallback = std::function<void(const std::string&)>;

public:
  std::string loadInitialValue(
    const rclcpp::Node::SharedPtr& _node,
    const std::string& _name,
    const StringCallback& _parent_callback = {},
    bool _default_continuous_value = false,
    double _default_timeout = 10.0);

private:
  bool getMainParameter();
  bool shouldPublish();
  bool waitForMessage(const rclcpp::Duration& _timeout);
  void stringCallback(const std_msgs::msg::String::ConstSharedPtr& _msg);

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  StringCallback parent_callback_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;

  std::string content_;
};
}  // namespace tobas::rviz
