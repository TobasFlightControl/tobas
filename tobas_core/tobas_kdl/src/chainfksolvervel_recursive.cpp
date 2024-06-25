#include "../include/tobas_kdl/chainfksolvervel_recursive.hpp"

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

  p_out_ = FrameVel::Identity();
  size_t j = 0;

  for (size_t i = 0; i < seg_nr; ++i)
  {
    const auto& seg = chain_.getSegment(i);

    // Calculate new Frame_base_ee
    if (seg.getJoint().type != Joint::Fixed)
    {
      p_out_ = p_out_ * FrameVel(seg.pose(q_in(j)), seg.twist(q_in(j), qd_in(j)));
      ++j;  // Only increase jointnr if the segment has a joint
    }
    else
    {
      p_out_ = p_out_ * FrameVel(seg.pose(0), seg.twist(0, 0));
    }
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
