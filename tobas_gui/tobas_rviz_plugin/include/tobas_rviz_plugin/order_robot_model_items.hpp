#pragma once

#include "./link_model.hpp"
#include "./joint_model_group.hpp"

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

struct OrderGroupsByName
{
  bool operator()(const JointModelGroup* a, const JointModelGroup* b) const
  {
    return a->getName() < b->getName();
  }
};
}  // namespace tobas
