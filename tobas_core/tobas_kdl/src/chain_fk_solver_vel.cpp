#include "../include/tobas_kdl/chain_fk_solver_vel.hpp"

using namespace std;

namespace kdl
{
ChainFkSolverVel_recursive::ChainFkSolverVel_recursive(const Chain& chain) : super(chain)
{
}

void ChainFkSolverVel_recursive::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
}

int ChainFkSolverVel_recursive::JntToCart(const JntArray& q_in, const JntArray& qd_in, int _seg_nr)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_ || qd_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const size_t seg_nr = _seg_nr >= 0 ? _seg_nr : chain_.getNrOfSegments();
  if (seg_nr > chain_.getNrOfSegments())
    return setDefaultError(E_OUT_OF_RANGE);

  p_out_.setIdentity();
  j_ = 0;

  for (size_t i = 0; i < seg_nr; ++i)
  {
    const auto& seg = chain_.getSegment(i);
    if (seg.joint().type != Joint::FIXED)
    {
      qj_ = q_in(j_);
      qdj_ = qd_in(j_);
      ++j_;  // Increase joint number only if the segment has a joint
    }
    else
    {
      qj_ = 0.;
      qdj_ = 0.;
    }
    p_out_ = p_out_ * FrameVel(seg.pose(qj_), seg.twist(qj_, qdj_));
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
