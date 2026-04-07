// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

#include "tobas_rviz_plugin/link_model.hpp"

namespace tobas
{
JointModel::JointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : name_(name), joint_index_(joint_index), first_variable_index_(first_variable_index)
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

const LinkModel* JointModel::getParentLinkModel() const
{
  return parent_link_model_;
}

const LinkModel* JointModel::getChildLinkModel() const
{
  return child_link_model_;
}

void JointModel::setParentLinkModel(const LinkModel* link)
{
  parent_link_model_ = link;
}

void JointModel::setChildLinkModel(const LinkModel* link)
{
  child_link_model_ = link;
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

size_t JointModel::getLocalVariableIndex(const std::string& variable) const
{
  const auto it = variable_index_map_.find(variable);
  if (it == variable_index_map_.end()) {
    throw std::runtime_error(
      "Could not find variable '" + variable + "' to get bounds for within joint '" + name_ + "'");
  }
  return it->second;
}

const JointModel* JointModel::getMimic() const
{
  return mimic_;
}

double JointModel::getMimicOffset() const
{
  return mimic_offset_;
}

double JointModel::getMimicFactor() const
{
  return mimic_factor_;
}

void JointModel::setMimic(const JointModel* mimic, double factor, double offset)
{
  mimic_ = mimic;
  mimic_factor_ = factor;
  mimic_offset_ = offset;
}

const std::vector<const JointModel*>& JointModel::getMimicRequests() const
{
  return mimic_requests_;
}

void JointModel::addMimicRequest(const JointModel* joint)
{
  mimic_requests_.push_back(joint);
}

void JointModel::addDescendantJointModel(const JointModel* joint)
{
  descendant_joint_models_.push_back(joint);
  if (joint->getType() != kFixed) {
    non_fixed_descendant_joint_models_.push_back(joint);
  }
}

void JointModel::addDescendantLinkModel(const LinkModel* link)
{
  descendant_link_models_.push_back(link);
}

const std::vector<const LinkModel*>& JointModel::getDescendantLinkModels() const
{
  return descendant_link_models_;
}

const std::vector<const JointModel*>& JointModel::getDescendantJointModels() const
{
  return descendant_joint_models_;
}

const std::vector<const JointModel*>& JointModel::getNonFixedDescendantJointModels() const
{
  return non_fixed_descendant_joint_models_;
}
}  // namespace tobas
