#include "../include/dh_kdl/chainfksolvervel_recursive.hpp"

using namespace std;

namespace KDL
{
ChainFkSolverVel_recursive::ChainFkSolverVel_recursive(const Chain& chain) : super(chain)
{
}

void ChainFkSolverVel_recursive::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
}

int ChainFkSolverVel_recursive::JntToCart(const JntArrayVel& in, int _seg_nr)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (in.q.rows() != nj_ || in.qdot.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const size_t seg_nr = _seg_nr >= 0 ? _seg_nr : chain_.getNrOfSegments();
  if (seg_nr > chain_.getNrOfSegments())
    return setDefaultError(E_OUT_OF_RANGE);

  p_out_ = FrameVel::Identity();
  size_t j = 0;

  for (size_t i = 0; i < seg_nr; ++i)
  {
    // Calculate new Frame_base_ee
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      p_out_ =
        p_out_
        * FrameVel(
          chain_.getSegment(i).pose(in.q(j)), chain_.getSegment(i).twist(in.q(j), in.qdot(j)));
      ++j;  // Only increase jointnr if the segment has a joint
    }
    else
    {
      p_out_ = p_out_ * FrameVel(chain_.getSegment(i).pose(0), chain_.getSegment(i).twist(0, 0));
    }
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace KDL
