#include "../include/dh_kdl/treefksolvervel.hpp"

using namespace std;

namespace KDL
{
TreeFkSolverVel::TreeFkSolverVel(const Tree& tree) : super(tree)
{
}

void TreeFkSolverVel::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
}

int TreeFkSolverVel::JntToCart(const JntArray& q, const JntArray& qd, const string& seg_name)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const auto it = tree_.getSegment(seg_name);
  if (it == tree_.getSegments().end())
  {
    error_msg_ = "Segment '" + seg_name + "' is not found in tree";
    return (error_code_ = E_NOT_FOUND);
  }

  p_out_ = recursiveFk(q, qd, it);
  return setDefaultError(E_NOERROR);
}

FrameVel TreeFkSolverVel::recursiveFk(
  const JntArray& q,
  const JntArray& qd,
  const SegmentMap::const_iterator& it)
{
  // Get the FraveVel for the current segment
  const auto& cur_ele = it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;

  const auto pose = cur_seg.pose(q(cur_idx));
  const auto twist = cur_seg.twist(q(cur_idx), qd(cur_idx));
  const FrameVel cur_framevel(pose, twist);

  const auto root_iter = tree_.getRootSegment();
  if (it == root_iter)
  {
    return cur_framevel;
  }
  else
  {
    const auto& parent_it = cur_ele.parent;
    return recursiveFk(q, qd, parent_it) * cur_framevel;
  }
}
}  // namespace KDL
