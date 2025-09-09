#include "tobas_kdl/tree_active_joints_extractor.hpp"

using namespace std;

namespace kdl
{
TreeActiveJointsExtractor::TreeActiveJointsExtractor(const Tree& tree) : super(tree)
{
}

int TreeActiveJointsExtractor::solve(const std::vector<std::string>& endpoints)
{
  if (!isUpToDate()) {
    return setDefaultError(kNotUpToDate);
  }

  active_joints_vec_.clear();
  active_joints_set_.clear();

  const auto root = tree_.getRootSegment();

  for (const auto& seg_name : endpoints) {
    if (!tree_.hasSegment(seg_name)) {
      return setDefaultError(kOutputRange);
    }

    auto it = tree_.getSegment(seg_name);
    while (it != root) {
      const auto& ele = it->second;
      const auto& joint = ele.segment.joint();
      if (joint.type != Joint::kFixed && !active_joints_set_.contains(joint.name)) {
        active_joints_vec_.push_back(joint.name);
        active_joints_set_.insert(joint.name);
      }
      it = ele.parent;
    }
  }

  return setDefaultError(kNoError);
}
}  // namespace kdl
