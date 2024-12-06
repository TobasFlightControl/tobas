#include "../include/tobas_kdl/tree_joint_axis_solver.hpp"

using namespace std;

namespace kdl
{
TreeJointAxisSolver::TreeJointAxisSolver(const Tree& tree) : super(tree), fk_solver_(tree)
{
}

void TreeJointAxisSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_solver_.updateInternalDataStructures();
}

int TreeJointAxisSolver::JntToCart(const JntArray& q_in, const string& seg_name)
{
  const auto cur_it = tree_.getSegment(seg_name);
  if (cur_it == tree_.getSegments().end())
    return setDefaultError(E_OUT_OF_RANGE);

  const auto& cur_ele = cur_it->second;
  const auto& cur_jnt = cur_ele.segment.joint();
  const auto& par_name = cur_ele.parent->first;

  if (fk_solver_.JntToCart(q_in, par_name) < 0)
    return copyError(fk_solver_);

  axis_out_ = fk_solver_.getFrame().M * cur_jnt.axis();
  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
