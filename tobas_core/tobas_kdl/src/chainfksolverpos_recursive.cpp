#include <iostream>

#include "../include/tobas_kdl/chainfksolverpos_recursive.hpp"

using namespace std;

namespace kdl
{
ChainFkSolverPos_recursive::ChainFkSolverPos_recursive(const Chain& chain) : super(chain)
{
}

void ChainFkSolverPos_recursive::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
}

int ChainFkSolverPos_recursive::JntToCart(const JntArray& q_in, int _seg_nr)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const size_t seg_nr = _seg_nr >= 0 ? _seg_nr : chain_.getNrOfSegments();
  if (seg_nr > chain_.getNrOfSegments())
    return setDefaultError(E_OUT_OF_RANGE);

  p_out_ = Frame::Identity();
  size_t j = 0;

  for (size_t i = 0; i < seg_nr; ++i)
  {
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      p_out_ = p_out_ * chain_.getSegment(i).pose(q_in(j));
      ++j;
    }
    else
    {
      p_out_ = p_out_ * chain_.getSegment(i).pose(0);
    }
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
