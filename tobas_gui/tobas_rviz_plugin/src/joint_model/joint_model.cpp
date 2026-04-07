// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

#include <algorithm>

#include "tobas_rviz_plugin/link_model.hpp"

namespace tobas
{
JointModel::JointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : type_(UNKNOWN)
  , parent_link_model_(nullptr)
  , child_link_model_(nullptr)
  , mimic_(nullptr)
  , mimic_factor_(1.)
  , mimic_offset_(0.)
  , name_(name)
  , joint_index_(joint_index)
  , first_variable_index_(first_variable_index)

{
}

JointModel::~JointModel() = default;

size_t JointModel::getLocalVariableIndex(const std::string& variable) const
{
  VariableIndexMap::const_iterator it = variable_index_map_.find(variable);
  if (it == variable_index_map_.end()) {
    throw std::runtime_error(
      "Could not find variable '" + variable + "' to get bounds for within joint '" + name_ + "'");
  }
  return it->second;
}

void JointModel::setMimic(const JointModel* mimic, double factor, double offset)
{
  mimic_ = mimic;
  mimic_factor_ = factor;
  mimic_offset_ = offset;
}

void JointModel::addMimicRequest(const JointModel* joint)
{
  mimic_requests_.push_back(joint);
}

void JointModel::addDescendantJointModel(const JointModel* joint)
{
  descendant_joint_models_.push_back(joint);
  if (joint->getType() != FIXED) {
    non_fixed_descendant_joint_models_.push_back(joint);
  }
}

void JointModel::addDescendantLinkModel(const LinkModel* link)
{
  descendant_link_models_.push_back(link);
}
}  // namespace tobas
