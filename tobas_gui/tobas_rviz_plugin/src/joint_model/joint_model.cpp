// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

#include "tobas_rviz_plugin/link_model.hpp"

namespace tobas
{
namespace rviz
{
namespace
{
/* Promote cached non-owning references for callers, omitting objects whose owners have released them. */
template <typename T>
std::vector<std::shared_ptr<const T>> lockAll(const std::vector<std::weak_ptr<const T>>& weak_ptrs)
{
  std::vector<std::shared_ptr<const T>> shared_ptrs;
  for (const auto& weak_ptr : weak_ptrs) {
    if (const auto shared_ptr = weak_ptr.lock()) {
      shared_ptrs.push_back(shared_ptr);
    }
  }
  return shared_ptrs;
}
}  // namespace

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

LinkModel::ConstSharedPtr JointModel::getChildLinkModel() const
{
  return child_link_model_;
}

void JointModel::setChildLinkModel(const LinkModel::ConstSharedPtr& link)
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

void JointModel::setMimic(const ConstSharedPtr& mimic, double factor, double offset)
{
  mimic_ = mimic;
  mimic_factor_ = factor;
  mimic_offset_ = offset;
}

std::vector<JointModel::ConstSharedPtr> JointModel::getMimicRequests() const
{
  return lockAll(mimic_requests_);
}

void JointModel::addMimicRequest(const ConstSharedPtr& joint)
{
  mimic_requests_.push_back(joint);
}

void JointModel::addDescendantJointModel(const ConstSharedPtr& joint)
{
  descendant_joint_models_.push_back(joint);
}

void JointModel::addDescendantLinkModel(const LinkModel::ConstSharedPtr& link)
{
  descendant_link_models_.push_back(link);
}

std::vector<LinkModel::ConstSharedPtr> JointModel::getDescendantLinkModels() const
{
  return lockAll(descendant_link_models_);
}

std::vector<JointModel::ConstSharedPtr> JointModel::getDescendantJointModels() const
{
  return lockAll(descendant_joint_models_);
}

}  // namespace rviz
}  // namespace tobas
