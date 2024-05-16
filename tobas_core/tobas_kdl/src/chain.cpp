#include "../include/tobas_kdl/chain.hpp"

namespace tobas_kdl
{
Chain::Chain() : segments(0), nj_(0), ns_(0)
{
}

Chain::Chain(const Chain& in) : segments(0), nj_(0), ns_(0)
{
  for (size_t i = 0; i < in.getNrOfSegments(); ++i)
    this->addSegment(in.getSegment(i));
}

Chain& Chain::operator=(const Chain& arg)
{
  nj_ = 0;
  ns_ = 0;
  segments.resize(0);
  for (size_t i = 0; i < arg.ns_; ++i)
    addSegment(arg.getSegment(i));
  return *this;
}

void Chain::addSegment(const Segment& segment)
{
  segments.push_back(segment);
  ++ns_;
  if (segment.getJoint().type != Joint::Fixed)
    ++nj_;
}

void Chain::addChain(const Chain& chain)
{
  for (size_t i = 0; i < chain.getNrOfSegments(); ++i)
    this->addSegment(chain.getSegment(i));
}
}  // namespace tobas_kdl
