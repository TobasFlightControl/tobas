// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

#include "./joint_model/fixed_joint_model.hpp"
#include "./joint_model/floating_joint_model.hpp"
#include "./joint_model/planar_joint_model.hpp"
#include "./joint_model/prismatic_joint_model.hpp"
#include "./joint_model/revolute_joint_model.hpp"
#include "./link_model.hpp"

namespace tobas
{
namespace rviz
{
/* Kinematic information required to display a URDF robot. */
class RobotModel
{
public:
  explicit RobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model);

  const std::string& getModelFrame() const;
  const urdf::ModelInterfaceSharedPtr& getURDF() const;
  JointModel::ConstSharedPtr getRootJoint() const;

  bool hasJointModel(const std::string& name) const;
  JointModel::ConstSharedPtr getJointModel(const std::string& name) const;
  JointModel::ConstSharedPtr getJointOfVariable(int variable_index) const;
  size_t getJointModelCount() const;

  LinkModel::ConstSharedPtr getRootLink() const;
  LinkModel::ConstSharedPtr getLinkModel(const std::string& name) const;
  size_t getLinkModelCount() const;

  void getVariableDefaultPositions(std::vector<double>& values) const;
  size_t getVariableCount() const;
  size_t getVariableIndex(const std::string& variable) const;

  JointModel::ConstSharedPtr
  getCommonRoot(const JointModel::ConstSharedPtr& a, const JointModel::ConstSharedPtr& b) const;

private:
  void updateMimicJoints(double* values) const;
  void buildModel(const urdf::ModelInterface& urdf_model);
  void buildMimic(const urdf::ModelInterface& urdf_model);
  void buildJointInfo();
  void computeDescendants();
  void computeCommonRoots();
  JointModel::SharedPtr buildRecursive(const LinkModel::SharedPtr& parent, const urdf::LinkConstSharedPtr& link);
  JointModel::SharedPtr constructJointModel(const urdf::LinkConstSharedPtr& child_link);
  LinkModel::SharedPtr constructLinkModel(const urdf::LinkConstSharedPtr& urdf_link);

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
}  // namespace rviz
}  // namespace tobas
