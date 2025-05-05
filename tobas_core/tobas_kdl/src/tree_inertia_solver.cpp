#include "tobas_kdl/tree_inertia_solver.hpp"

using namespace std;

namespace kdl
{
TreeInertiaSolver::TreeInertiaSolver(const Tree& tree) : super(tree)
{
  initialize();
}

bool TreeInertiaSolver::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  initialize();

  return true;
}

int TreeInertiaSolver::JntToCart(const JntArray& q)
{
  if (!isUpToDate()) {
    return setDefaultError(E_NOT_UP_TO_DATE);
  }
  if (q.rows() != nj_) {
    return setDefaultError(E_SIZE_MISMATCH);
  }

  const auto root_it = tree_.getRootSegment();
  step(root_it, q);

  return setDefaultError(E_NOERROR);
}

void TreeInertiaSolver::initialize()
{
  X_.clear();
  I_.clear();
  for (const auto& [seg_name, _] : tree_.getSegments()) {
    X_[seg_name] = Frame::Identity();
    I_[seg_name] = RigidBodyInertia::Zero();
  }
}

void TreeInertiaSolver::step(const SegmentMap::const_iterator& cur_it, const JntArray& q)
{
  const auto& cur_name = cur_it->first;
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& par_it = cur_ele.parent;
  const auto& par_name = par_it->first;

  // Forward calculation
  const auto& qj = cur_seg.joint().type == Joint::FIXED ? 0. : q(cur_ele.q_nr);
  X_.at(cur_name) = cur_seg.pose(qj);
  I_.at(cur_name) = cur_seg.inertia();

  // Propagate calculations over each child segment
  for (const auto& child_it : cur_ele.children) {
    step(child_it, q);
  }

  // Backward calculation
  if (cur_it != tree_.getRootSegment()) {
    I_.at(par_name) += X_.at(cur_name) * I_.at(cur_name);
  }
}
}  // namespace kdl
