// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model/joint_model.hpp"
#include "./link_model.hpp"

namespace tobas
{
namespace rviz
{
struct OrderLinksByIndex
{
  bool operator()(const LinkModel::ConstSharedPtr& _a, const LinkModel::ConstSharedPtr& _b) const
  {
    return _a->getLinkIndex() < _b->getLinkIndex();
  }
};

struct OrderJointsByIndex
{
  bool operator()(const JointModel::ConstSharedPtr& _a, const JointModel::ConstSharedPtr& _b) const
  {
    return _a->getJointIndex() < _b->getJointIndex();
  }
};
}  // namespace rviz
}  // namespace tobas
