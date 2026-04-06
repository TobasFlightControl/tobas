// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./link_model.hpp"

namespace tobas
{
struct OrderLinksByIndex
{
  bool operator()(const LinkModel* a, const LinkModel* b) const
  {
    return a->getLinkIndex() < b->getLinkIndex();
  }
};

struct OrderJointsByIndex
{
  bool operator()(const JointModel* a, const JointModel* b) const
  {
    return a->getJointIndex() < b->getJointIndex();
  }
};
}  // namespace tobas
