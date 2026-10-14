// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/src/joint_model.cpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2008-2013, Willow Garage, Inc.
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

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

#include "tobas_rviz_plugin/link_model.hpp"

namespace tobas::rviz
{
namespace
{
/** Promote cached non-owning references for callers, omitting objects whose owners have released them. */
template <typename T>
std::vector<std::shared_ptr<const T>> lockAll(const std::vector<std::weak_ptr<const T>>& _weak_ptrs)
{
  std::vector<std::shared_ptr<const T>> shared_ptrs;
  for (const auto& weak_ptr : _weak_ptrs) {
    if (const auto shared_ptr = weak_ptr.lock()) {
      shared_ptrs.push_back(shared_ptr);
    }
  }
  return shared_ptrs;
}
}  // namespace

JointModel::JointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : name_(_name), joint_index_(_joint_index), first_variable_index_(_first_variable_index)
{
}

JointModel::~JointModel() = default;

const std::string& JointModel::getName() const
{
  return name_;
}

JointModel::JointType JointModel::getType() const
{
  return type_;
}

LinkModel::ConstSharedPtr JointModel::getChildLinkModel() const
{
  return child_link_model_;
}

void JointModel::setChildLinkModel(const LinkModel::ConstSharedPtr& _link)
{
  child_link_model_ = _link;
}

const std::vector<std::string>& JointModel::getVariableNames() const
{
  return variable_names_;
}

size_t JointModel::getVariableCount() const
{
  return variable_names_.size();
}

size_t JointModel::getFirstVariableIndex() const
{
  return first_variable_index_;
}

size_t JointModel::getJointIndex() const
{
  return joint_index_;
}

JointModel::ConstSharedPtr JointModel::getMimic() const
{
  return mimic_.lock();
}

double JointModel::getMimicOffset() const
{
  return mimic_offset_;
}

double JointModel::getMimicFactor() const
{
  return mimic_factor_;
}

void JointModel::setMimic(const ConstSharedPtr& _mimic, double _factor, double _offset)
{
  mimic_ = _mimic;
  mimic_factor_ = _factor;
  mimic_offset_ = _offset;
}

std::vector<JointModel::ConstSharedPtr> JointModel::getMimicRequests() const
{
  return lockAll(mimic_requests_);
}

void JointModel::addMimicRequest(const ConstSharedPtr& _joint)
{
  mimic_requests_.push_back(_joint);
}

void JointModel::addDescendantJointModel(const ConstSharedPtr& _joint)
{
  descendant_joint_models_.push_back(_joint);
}

void JointModel::addDescendantLinkModel(const LinkModel::ConstSharedPtr& _link)
{
  descendant_link_models_.push_back(_link);
}

std::vector<LinkModel::ConstSharedPtr> JointModel::getDescendantLinkModels() const
{
  return lockAll(descendant_link_models_);
}

std::vector<JointModel::ConstSharedPtr> JointModel::getDescendantJointModels() const
{
  return lockAll(descendant_joint_models_);
}

}  // namespace tobas::rviz
