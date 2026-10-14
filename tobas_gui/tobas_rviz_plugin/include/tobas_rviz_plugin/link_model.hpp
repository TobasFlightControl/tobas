// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/include/moveit/robot_model/link_model.hpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

namespace tobas::rviz
{
class JointModel;

/** A link in the robot kinematic tree. */
class LinkModel
{
public:
  using SharedPtr = std::shared_ptr<LinkModel>;
  using ConstSharedPtr = std::shared_ptr<const LinkModel>;

  explicit LinkModel(const std::string& _name, size_t _link_index);

  const std::string& getName() const;
  size_t getLinkIndex() const;

  std::shared_ptr<const JointModel> getParentJointModel() const;
  void setParentJointModel(const std::shared_ptr<const JointModel>& _joint);

  ConstSharedPtr getParentLinkModel() const;
  void setParentLinkModel(const ConstSharedPtr& _link);

  const std::vector<std::shared_ptr<const JointModel>>& getChildJointModels() const;
  void addChildJointModel(const std::shared_ptr<const JointModel>& _joint);

  const Eigen::Isometry3d& getJointOriginTransform() const;
  bool jointOriginTransformIsIdentity() const;
  bool parentJointIsFixed() const;
  void setJointOriginTransform(const Eigen::Isometry3d& _transform);

private:
  const std::string name_;
  const size_t link_index_;

  /** Parent references are non-owning to avoid cycles with the owning child relationships. */
  std::weak_ptr<const JointModel> parent_joint_model_;
  std::weak_ptr<const LinkModel> parent_link_model_;

  std::vector<std::shared_ptr<const JointModel>> child_joint_models_;
  bool is_parent_joint_fixed_ = false;
  bool joint_origin_transform_is_identity_ = true;
  Eigen::Isometry3d joint_origin_transform_ = Eigen::Isometry3d::Identity();
};
}  // namespace tobas::rviz
