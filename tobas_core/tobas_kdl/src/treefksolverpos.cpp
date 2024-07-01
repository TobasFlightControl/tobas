#include "../include/tobas_kdl/treefksolverpos.hpp"

using namespace std;

namespace kdl
{
TreeFkSolverPos::TreeFkSolverPos(const Tree& tree) : super(tree)
{
}

void TreeFkSolverPos::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
}

int TreeFkSolverPos::JntToCart(const JntArray& q, const string& seg_name)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const auto it = tree_.getSegment(seg_name);
  if (it == tree_.getSegments().end())
  {
    error_msg_ = "Segment '" + seg_name + "' is not found in tree";
    return (error_code_ = E_NOT_FOUND);
  }

  p_out_ = recursiveFk(q, it);
  return setDefaultError(E_NOERROR);
}

Frame TreeFkSolverPos::recursiveFk(const JntArray& q, const SegmentMap::const_iterator& it)
{
  // Get the Frame for the current segment
  const auto& cur_ele = it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;
  const auto& cur_frame = cur_seg.pose(q(cur_idx));

  const auto root_it = tree_.getRootSegment();
  if (it == root_it)
  {
    return cur_frame;
  }
  else
  {
    const auto& parent_it = cur_ele.parent;
    return recursiveFk(q, parent_it) * cur_frame;
  }
}
}  // namespace kdl
