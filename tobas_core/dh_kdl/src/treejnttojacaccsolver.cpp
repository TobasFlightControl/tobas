#include "../include/dh_kdl/treejnttojacaccsolver.hpp"

namespace KDL
{
TreeJntToJacAccSolver::TreeJntToJacAccSolver(const Tree& tree) : super(tree)
{
  updateInternalDataStructures();
}

void TreeJntToJacAccSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  for (const auto& [seg_name, _] : tree_.getSegments())
  {
    R_[seg_name] = Rotation();
    v_[seg_name] = Twist();
    a_[seg_name] = Accel();
    Jdqd_out_[seg_name] = Accel();
  }
}

int TreeJntToJacAccSolver::JntToCart(const JntArray& q, const JntArray& qd)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  JntToCartRec(tree_.getRootSegment(), q, qd);

  return setDefaultError(E_NOERROR);
}

void TreeJntToJacAccSolver::JntToCartRec(
  const SegmentMap::const_iterator& segment,
  const JntArray& q,
  const JntArray& qd)
{
  const auto& seg = segment->second.segment;
  const auto& seg_name = segment->first;
  const auto& par_name = segment->second.parent->first;

  // Do forward calculations
  const auto& j = segment->second.q_nr;
  double qj, qdj;
  if (seg.getJoint().type != Joint::Fixed)
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
}  // namespace KDL
