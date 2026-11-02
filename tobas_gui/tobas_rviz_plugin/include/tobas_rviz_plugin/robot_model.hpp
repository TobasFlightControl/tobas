// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/include/moveit/robot_model/robot_model.hpp
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.hpp>

#include "./joint_model/joint_model.hpp"
#include "./link_model.hpp"

namespace tobas::rviz
{
/** Kinematic information required to display a URDF robot. */
class RobotModel
{
public:
  explicit RobotModel(const urdf::ModelInterfaceSharedPtr& _urdf_model);

  const std::string& getModelFrame() const;
  const urdf::ModelInterfaceSharedPtr& getURDF() const;
  JointModel::ConstSharedPtr getRootJoint() const;

  bool hasJointModel(const std::string& _name) const;
  JointModel::ConstSharedPtr getJointModel(const std::string& _name) const;
  JointModel::ConstSharedPtr getJointOfVariable(int _variable_index) const;
  size_t getJointModelCount() const;

  LinkModel::ConstSharedPtr getRootLink() const;
  LinkModel::ConstSharedPtr getLinkModel(const std::string& _name) const;
  size_t getLinkModelCount() const;

  void getVariableDefaultPositions(std::vector<double>& _values) const;
  size_t getVariableCount() const;
  size_t getVariableIndex(const std::string& _variable) const;

  JointModel::ConstSharedPtr
  getCommonRoot(const JointModel::ConstSharedPtr& _a, const JointModel::ConstSharedPtr& _b) const;

private:
  void updateMimicJoints(double* _values) const;
  void buildModel(const urdf::ModelInterface& _urdf_model);
  void buildMimic(const urdf::ModelInterface& _urdf_model);
  void buildJointInfo();
  void computeDescendants();
  void computeCommonRoots();
  JointModel::SharedPtr buildRecursive(const LinkModel::SharedPtr& _parent, const urdf::LinkConstSharedPtr& _link);
  JointModel::SharedPtr constructJointModel(const urdf::LinkConstSharedPtr& _child_link);
  LinkModel::SharedPtr constructLinkModel(const urdf::LinkConstSharedPtr& _urdf_link);

  std::string model_name_;
  std::string model_frame_;
  urdf::ModelInterfaceSharedPtr urdf_;

  LinkModel::SharedPtr root_link_;
  std::map<std::string, LinkModel::SharedPtr> link_model_map_;
  std::vector<LinkModel::SharedPtr> link_model_vector_;

  JointModel::SharedPtr root_joint_;
  JointModelMap joint_model_map_;
  std::vector<JointModel::SharedPtr> joint_model_vector_;
  std::vector<JointModel::SharedPtr> active_joint_model_vector_;
  std::vector<JointModel::ConstSharedPtr> mimic_joints_;
  std::vector<int> common_joint_roots_;

  size_t variable_count_ = 0;
  std::map<std::string, size_t> joint_variables_index_map_;
  std::vector<int> active_joint_model_start_index_;
  std::vector<JointModel::ConstSharedPtr> joints_of_variable_;
};
}  // namespace tobas::rviz
