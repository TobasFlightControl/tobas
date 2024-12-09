#include "../include/tobas_kdl/tree_fk_solver_vel.hpp"

using namespace std;

namespace kdl
{
TreeFkSolverVel::TreeFkSolverVel(const Tree& tree) : super(tree)
{
}

int TreeFkSolverVel::JntToCart(const JntArray& q, const JntArray& qd, const string& seg_name)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);
  if (!tree_.hasSegment(seg_name))
    return setDefaultError(E_OUT_OF_RANGE);

  const auto seg_it = tree_.getSegment(seg_name);
  p_out_ = recursiveFk(q, qd, seg_it);
  return setDefaultError(E_NOERROR);
}

FrameVel TreeFkSolverVel::recursiveFk(const JntArray& q, const JntArray& qd, const SegmentMap::const_iterator& cur_it)
{
  // Get the FraveVel for the current segment
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;

  const auto pose = cur_seg.pose(q(cur_idx));
  const auto twist = cur_seg.twist(q(cur_idx), qd(cur_idx));
  const FrameVel cur_framevel(pose, twist);

  if (cur_it == tree_.getRootSegment())
    return cur_framevel;

  const auto& parent_it = cur_ele.parent;
  return recursiveFk(q, qd, parent_it) * cur_framevel;
}
}  // namespace kdl
