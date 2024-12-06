#include "../include/tobas_kdl/tree_fk_solver_pos_all.hpp"

using namespace std;

namespace kdl
{
TreeFkSolverPosAll::TreeFkSolverPosAll(const Tree& tree) : super(tree)
{
}

void TreeFkSolverPosAll::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  frames_.clear();
}

int TreeFkSolverPosAll::JntToCart(const JntArray& q)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const auto root_it = tree_.getRootSegment();
  const auto& root_name = root_it->first;
  const auto& root_ele = root_it->second;

  frames_[root_name] = Frame::Identity();
  for (const auto& child_it : root_ele.children)
    recursiveFk(q, frames_.at(root_name), child_it);

  return setDefaultError(E_NOERROR);
}

void TreeFkSolverPosAll::recursiveFk(
  const JntArray& q,
  const Frame& par_frame,
  const SegmentMap::const_iterator& cur_it)
{
  // Get the current segment
  const auto& cur_name = cur_it->first;
  const auto& cur_ele = cur_it->second;

  // Fill the frame for the current segment
  frames_[cur_name] = par_frame * cur_ele.segment.pose(q(cur_ele.q_nr));

  // Spread to the children
  for (const auto& child_it : cur_ele.children)
    recursiveFk(q, frames_.at(cur_name), child_it);
}
}  // namespace kdl
