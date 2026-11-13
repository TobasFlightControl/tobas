// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
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

#include "./robot_model.hpp"

namespace tobas::rviz
{
/** Joint positions and the link transforms derived from them. */
class RobotState
{
public:
  explicit RobotState(const RobotModel& _robot_model);

  const RobotModel& getRobotModel() const;

  void
  setVariablePositions(const std::vector<std::string>& _variable_names, const std::vector<double>& _variable_positions);

  void setJointPositions(const std::string& _joint_name, const Eigen::Isometry3d& _transform);

  void setToDefaultValues();
  void update(bool _force = false);

  const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel::ConstSharedPtr& _link) const;

private:
  void markDirtyJointTransforms(const JointModel::ConstSharedPtr& _joint);
  void updateMimicJoint(const JointModel::ConstSharedPtr& _joint);
  void updateLinkTransforms();
  void updateLinkTransformsInternal(const JointModel::ConstSharedPtr& _start);
  const Eigen::Isometry3d& getJointTransform(const JointModel::ConstSharedPtr& _joint);

  bool checkLinkTransforms() const;

  const RobotModel& robot_model_;
  std::vector<double> positions_;
  JointModel::ConstSharedPtr dirty_link_transforms_;
  std::vector<Eigen::Isometry3d> variable_joint_transforms_;
  std::vector<Eigen::Isometry3d> global_link_transforms_;
  std::vector<uint8_t> dirty_joint_transforms_;
};
}  // namespace tobas::rviz
