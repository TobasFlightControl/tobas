#include "../include/tobas_kdl/treeactivejointsextractor.hpp"

using namespace std;

namespace KDL
{
TreeActiveJointsExtractor::TreeActiveJointsExtractor(const Tree& tree) : super(tree)
{
}

void TreeActiveJointsExtractor::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
}

int TreeActiveJointsExtractor::solve(const std::vector<std::string>& endpoints)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);

  active_joints_vec_.clear();
  active_joints_set_.clear();

  const auto root = tree_.getRootSegment();

  for (const auto& seg_name : endpoints)
  {
    auto it = tree_.getSegment(seg_name);

    if (it == tree_.getSegments().end())
    {
      error_msg_ = "Segment '" + seg_name + "' is not found in tree.";
      return (error_code_ = E_NOT_FOUND);
    }

    while (it != root)
    {
      const auto& joint = it->second.segment.getJoint();
      if (joint.type != Joint::Fixed && !tobas_std::contains(active_joints_set_, joint.name))
      {
        active_joints_vec_.push_back(joint.name);
        active_joints_set_.insert(joint.name);
      }
      it = it->second.parent;
    }
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace KDL
