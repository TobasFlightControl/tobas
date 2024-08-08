#include "../include/tobas_kdl/treejntaxissolver.hpp"

using namespace std;

namespace kdl
{
TreeJntAxisSolver::TreeJntAxisSolver(const Tree& tree) : super(tree), fk_solver_(tree)
{
}

void TreeJntAxisSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_solver_.updateInternalDataStructures();
}

int TreeJntAxisSolver::JntToCart(const JntArray& q_in, const string& seg_name)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const auto& cur_it = tree_.getSegment(seg_name);
  const auto& cur_ele = cur_it->second;
  const auto& cur_jnt = cur_ele.segment.joint();
  const auto& par_name = cur_ele.parent->first;

  if (fk_solver_.JntToCart(q_in, par_name) < 0)
    return copyError(fk_solver_);

  axis_out_ = fk_solver_.getFrame().M * cur_jnt.axis();
  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
