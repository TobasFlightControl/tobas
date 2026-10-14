// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_ros/planning/rdf_loader/src/rdf_loader.cpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Mathias Lüdtke, Dave Coleman */

#include "tobas_rviz_plugin/rdf_loader.hpp"

#include <QDebug>

namespace tobas::rviz
{
RDFLoader::RDFLoader(
  const rclcpp::Node::SharedPtr& _node,
  const std::string& _ros_name,
  bool _default_continuous_value,
  double _default_timeout)
  : ros_name_(_ros_name)
{
  const auto start = _node->now();

  urdf_string_ = urdf_ssp_.loadInitialValue(
    _node,
    _ros_name,
    [this](const std::string& _new_urdf_string) { return urdfUpdateCallback(_new_urdf_string); },
    _default_continuous_value,
    _default_timeout);

  if (!loadFromStrings()) {
    return;
  }

  qInfo() << "Loaded robot model in" << (_node->now() - start).seconds() << "seconds.";
}

const urdf::ModelInterfaceSharedPtr& RDFLoader::getURDF() const
{
  return urdf_;
}

void RDFLoader::setNewModelCallback(const NewModelCallback& _cb)
{
  new_model_cb_ = _cb;
}

bool RDFLoader::loadFromStrings()
{
  auto urdf = std::make_unique<urdf::Model>();
  if (!urdf->initString(urdf_string_)) {
    qCritical() << "Failed to parse URDF.";
    return false;
  }

  urdf_ = std::move(urdf);
  return true;
}

void RDFLoader::urdfUpdateCallback(const std::string& _new_urdf_string)
{
  urdf_string_ = _new_urdf_string;
  if (!loadFromStrings()) {
    return;
  }
  if (new_model_cb_) {
    new_model_cb_();
  }
}
}  // namespace tobas::rviz
