// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/include/moveit/robot_model/joint_model.hpp
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

namespace tobas::rviz
{
class LinkModel;
class JointModel;

/** Map of names to instances for JointModel. */
using JointModelMap = std::map<std::string, std::shared_ptr<JointModel>>;

/** A joint from the robot. Models the transform that this joint applies in the kinematic chain. */
class JointModel
{
public:
  using SharedPtr = std::shared_ptr<JointModel>;
  using ConstSharedPtr = std::shared_ptr<const JointModel>;

  /** The different types of joints we support. */
  enum JointType
  {
    kUnknown,
    kRevolute,
    kPrismatic,
    kPlanar,
    kFloating,
    kFixed,
  };

  explicit JointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index);
  virtual ~JointModel();

  /** Get the name of the joint. */
  const std::string& getName() const;

  /** Get the type of joint. */
  JointType getType() const;

  /** Get the link that this joint connects to. */
  std::shared_ptr<const LinkModel> getChildLinkModel() const;

  void setChildLinkModel(const std::shared_ptr<const LinkModel>& _link);

  /** Get the names of the variables that make up this joint, in the order they appear in corresponding states. */
  const std::vector<std::string>& getVariableNames() const;

  /** Get the number of variables that describe this joint. */
  size_t getVariableCount() const;

  /** Get the index of this joint's first variable within the full robot state. */
  size_t getFirstVariableIndex() const;

  /** Get the index of this joint within the robot model. */
  size_t getJointIndex() const;

  /** Provide a default value for the joint given the joint variable bounds. */
  virtual void getVariableDefaultPositions(double* _values) const = 0;

  /** Get the joint this one is mimicking. */
  ConstSharedPtr getMimic() const;

  /** If mimicking a joint, this is the offset added to that joint's value. */
  double getMimicOffset() const;

  /** If mimicking a joint, this is the multiplicative factor for that joint's value. */
  double getMimicFactor() const;

  /** Mark this joint as mimicking `mimic` using `factor` and `offset`. */
  void setMimic(const ConstSharedPtr& _mimic, double _factor, double _offset);

  /** The joint models whose values would be modified if the value of this joint changed. */
  std::vector<ConstSharedPtr> getMimicRequests() const;

  /** Notify this joint that there is another joint that mimics it. */
  void addMimicRequest(const ConstSharedPtr& _joint);
  void addDescendantJointModel(const ConstSharedPtr& _joint);
  void addDescendantLinkModel(const std::shared_ptr<const LinkModel>& _link);

  /** Get all the link models that descend from this joint, in the kinematic tree. */
  std::vector<std::shared_ptr<const LinkModel>> getDescendantLinkModels() const;

  /** Get all the joint models that descend from this joint, in the kinematic tree. */
  std::vector<ConstSharedPtr> getDescendantJointModels() const;

  /** Given the joint values for a joint, compute the corresponding transform. */
  virtual void computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const = 0;

  /** Given the transform generated by joint, compute the corresponding joint values. */
  virtual void computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const = 0;

protected:
  JointType type_ = kUnknown;

  std::vector<std::string> variable_names_;  ///< The full names to use for the variables that make up this joint

  std::shared_ptr<const LinkModel> child_link_model_;

  /** Non-owning because the mimicked joint also records this joint as a mimic request. */
  std::weak_ptr<const JointModel> mimic_;

  double mimic_factor_ = 1.0;  ///< The multiplier to the mimic joint

  double mimic_offset_ = 0.0;  ///< The offset to the mimic joint

  /** Non-owning reverse references to prevent cycles between mimicking joints. */
  std::vector<std::weak_ptr<const JointModel>> mimic_requests_;

  /** Non-owning caches; the robot model and kinematic tree own these descendants. */
  std::vector<std::weak_ptr<const LinkModel>> descendant_link_models_;

  /** Non-owning cache of joints that follow this one, including mimic joints. */
  std::vector<std::weak_ptr<const JointModel>> descendant_joint_models_;

private:
  const std::string name_;

  const size_t joint_index_;  ///< Index for this joint in the array of joints of the complete model

  const size_t first_variable_index_;  ///< The index of this joint's first variable, in the complete robot state
};
}  // namespace tobas::rviz
