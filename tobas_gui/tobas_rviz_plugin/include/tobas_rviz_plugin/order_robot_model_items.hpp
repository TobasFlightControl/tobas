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
  bool operator()(const LinkModel::ConstSharedPtr& a, const LinkModel::ConstSharedPtr& b) const
  {
    return a->getLinkIndex() < b->getLinkIndex();
  }
};

struct OrderJointsByIndex
{
  bool operator()(const JointModel::ConstSharedPtr& a, const JointModel::ConstSharedPtr& b) const
  {
    return a->getJointIndex() < b->getJointIndex();
  }
};
}  // namespace rviz
}  // namespace tobas
