#include "../include/tobas_kdl/tree_jac_acc_solver.hpp"

namespace kdl
{
TreeJacAccSolver::TreeJacAccSolver(const Tree& tree) : super(tree)
{
  initialize();
}

bool TreeJacAccSolver::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures())
    return false;

  initialize();

  return true;
}

int TreeJacAccSolver::JntToCart(const JntArray& q, const JntArray& qd)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  JntToCartRec(tree_.getRootSegment(), q, qd);

  return setDefaultError(E_NOERROR);
}

void TreeJacAccSolver::initialize()
{
  for (const auto& [seg_name, _] : tree_.getSegments())
  {
    R_[seg_name] = Rotation::Identity();
    v_[seg_name] = Twist::Zero();
    a_[seg_name] = Accel::Zero();
    Jdqd_out_[seg_name] = Accel::Zero();
  }
}

void TreeJacAccSolver::JntToCartRec(const SegmentMap::const_iterator& segment, const JntArray& q, const JntArray& qd)
{
  const auto& seg = segment->second.segment;
  const auto& seg_name = segment->first;
  const auto& par_name = segment->second.parent->first;

  // Do forward calculations
  const auto& j = segment->second.q_nr;
  double qj, qdj;
  if (seg.joint().type != Joint::FIXED)
  {
    qj = q(j);
    qdj = qd(j);
  }
  else
  {
    qj = qdj = 0;
  }

  const auto Xj = seg.pose(qj);
  const auto vj = Xj.M.inverse(seg.twist(qj, qdj));  // Transform velocity

  if (segment == tree_.getRootSegment())
  {
    R_.at(seg_name) = Rotation::Identity();
    v_.at(seg_name) = vj;
    a_.at(seg_name) = vj * vj;
  }
  else
  {
    R_.at(seg_name) = R_.at(par_name) * Xj.M;
    v_.at(seg_name) = Xj.inverse(v_.at(par_name)) + vj;
    a_.at(seg_name) = Xj.inverse(a_.at(par_name)) + v_.at(seg_name) * vj;
  }

  // Calculate Jdqd wrt. the root frame
  Jdqd_out_.at(seg_name) = R_.at(seg_name) * a_.at(seg_name);

  // propagate calculations over each child segment
  for (const auto& child : segment->second.children)
    JntToCartRec(child, q, qd);
}
}  // namespace kdl
