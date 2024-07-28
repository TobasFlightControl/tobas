#include "../include/tobas_kdl/treefksolvervel.hpp"

using namespace std;

namespace kdl
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
  if (!tree_.hasSegment(seg_name))
    return setDefaultError(E_OUT_OF_RANGE);

  const auto it = tree_.getSegment(seg_name);
  p_out_ = recursiveFk(q, qd, it);
  return setDefaultError(E_NOERROR);
}

FrameVel TreeFkSolverVel::recursiveFk(const JntArray& q, const JntArray& qd, const SegmentMap::const_iterator& it)
{
  // Get the FraveVel for the current segment
  const auto& cur_ele = it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;

  const auto pose = cur_seg.pose(q(cur_idx));
  const auto twist = cur_seg.twist(q(cur_idx), qd(cur_idx));
  const FrameVel cur_framevel(pose, twist);

  const auto root_it = tree_.getRootSegment();
  if (it == root_it)
  {
    return cur_framevel;
  }
  else
  {
    const auto& parent_it = cur_ele.parent;
    return recursiveFk(q, qd, parent_it) * cur_framevel;
  }
}
}  // namespace kdl
