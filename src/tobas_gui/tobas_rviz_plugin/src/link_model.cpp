// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/src/link_model.cpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "tobas_rviz_plugin/link_model.hpp"

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

namespace tobas::rviz
{
LinkModel::LinkModel(const std::string& _name, size_t _link_index) : name_(_name), link_index_(_link_index)
{
}

const std::string& LinkModel::getName() const
{
  return name_;
}

size_t LinkModel::getLinkIndex() const
{
  return link_index_;
}

JointModel::ConstSharedPtr LinkModel::getParentJointModel() const
{
  return parent_joint_model_.lock();
}

void LinkModel::setParentJointModel(const JointModel::ConstSharedPtr& _joint)
{
  parent_joint_model_ = _joint;
  is_parent_joint_fixed_ = _joint->getType() == JointModel::kFixed;
}

LinkModel::ConstSharedPtr LinkModel::getParentLinkModel() const
{
  return parent_link_model_.lock();
}

void LinkModel::setParentLinkModel(const ConstSharedPtr& _link)
{
  parent_link_model_ = _link;
}

const std::vector<JointModel::ConstSharedPtr>& LinkModel::getChildJointModels() const
{
  return child_joint_models_;
}

void LinkModel::addChildJointModel(const JointModel::ConstSharedPtr& _joint)
{
  child_joint_models_.push_back(_joint);
}

const Eigen::Isometry3d& LinkModel::getJointOriginTransform() const
{
  return joint_origin_transform_;
}

bool LinkModel::jointOriginTransformIsIdentity() const
{
  return joint_origin_transform_is_identity_;
}

bool LinkModel::parentJointIsFixed() const
{
  return is_parent_joint_fixed_;
}

void LinkModel::setJointOriginTransform(const Eigen::Isometry3d& _transform)
{
  joint_origin_transform_ = _transform;
  joint_origin_transform_is_identity_ =
    joint_origin_transform_.linear().isIdentity() &&
    joint_origin_transform_.translation().norm() < std::numeric_limits<double>::epsilon();
}
}  // namespace tobas::rviz
