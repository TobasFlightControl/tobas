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
  ~RobotModel();

  const std::string& getModelFrame() const;
  const urdf::ModelInterfaceSharedPtr& getURDF() const;
  const JointModel* getRootJoint() const;

  bool hasJointModel(const std::string& name) const;
  const JointModel* getJointModel(const std::string& name) const;
  const JointModel* getJointOfVariable(int variable_index) const;
  size_t getJointModelCount() const;

  const LinkModel* getRootLink() const;
  const LinkModel* getLinkModel(const std::string& name) const;
  size_t getLinkModelCount() const;

  void getVariableDefaultPositions(std::vector<double>& values) const;
  size_t getVariableCount() const;
  size_t getVariableIndex(const std::string& variable) const;

  const JointModel* getCommonRoot(const JointModel* a, const JointModel* b) const;

private:
  void updateMimicJoints(double* values) const;
  void buildModel(const urdf::ModelInterface& urdf_model);
  void buildMimic(const urdf::ModelInterface& urdf_model);
  void buildJointInfo();
  void computeDescendants();
  void computeCommonRoots();
  JointModel* buildRecursive(LinkModel* parent, const urdf::Link* link);
  JointModel* constructJointModel(const urdf::Link* child_link);
  LinkModel* constructLinkModel(const urdf::Link* urdf_link);

  std::string model_name_;
  std::string model_frame_;
  urdf::ModelInterfaceSharedPtr urdf_;

  const LinkModel* root_link_ = nullptr;
  std::map<std::string, LinkModel*> link_model_map_;
  std::vector<LinkModel*> link_model_vector_;

  const JointModel* root_joint_ = nullptr;
  JointModelMap joint_model_map_;
  std::vector<JointModel*> joint_model_vector_;
  std::vector<JointModel*> active_joint_model_vector_;
  std::vector<const JointModel*> mimic_joints_;
  std::vector<int> common_joint_roots_;

  size_t variable_count_ = 0;
  std::map<std::string, size_t> joint_variables_index_map_;
  std::vector<int> active_joint_model_start_index_;
  std::vector<const JointModel*> joints_of_variable_;
};
}  // namespace rviz
}  // namespace tobas
