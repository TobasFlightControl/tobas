#include "../include/tobas_kdl/tree_fk_solver_pos.hpp"

using namespace std;

namespace kdl
{
TreeFkSolverPos::TreeFkSolverPos(const Tree& tree) : super(tree)
{
}

int TreeFkSolverPos::JntToCart(const JntArray& q, const string& seg_name)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);
  if (!tree_.hasSegment(seg_name))
    return setDefaultError(E_OUT_OF_RANGE);

  const auto seg_it = tree_.getSegment(seg_name);
  p_out_ = recursiveFk(q, seg_it);
  return setDefaultError(E_NOERROR);
}

Frame TreeFkSolverPos::recursiveFk(const JntArray& q, const SegmentMap::const_iterator& cur_it)
{
  // Get the Frame for the current segment
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;
  const auto& cur_frame = cur_seg.pose(q(cur_idx));

  if (cur_it == tree_.getRootSegment())
    return cur_frame;

  const auto& parent_it = cur_ele.parent;
  return recursiveFk(q, parent_it) * cur_frame;
}
}  // namespace kdl
