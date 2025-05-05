#include "tobas_kdl/chain.hpp"

using namespace std;

namespace kdl
{
Chain::Chain()
{
}

Chain::Chain(const Chain& in)
{
  for (size_t i = 0; i < in.getNrOfSegments(); ++i) {
    addSegment(in.getSegment(i));
  }
}

Chain& Chain::operator=(const Chain& arg)
{
  clear();

  for (size_t i = 0; i < arg.ns_; ++i) {
    addSegment(arg.getSegment(i));
  }
  return *this;
}

void Chain::clear()
{
  nj_ = 0;
  ns_ = 0;
  segments.clear();
}

void Chain::addSegment(const Segment& segment)
{
  segments.push_back(segment);
  ++ns_;
  if (segment.joint().type != Joint::FIXED) {
    ++nj_;
  }
}

void Chain::addChain(const Chain& chain)
{
  for (size_t i = 0; i < chain.getNrOfSegments(); ++i) {
    addSegment(chain.getSegment(i));
  }
}

ostream& operator<<(ostream& os, const Chain& arg)
{
  for (const auto& seg : arg.segments) {
    os << seg.name() << endl;
  }
  return os;
}
}  // namespace kdl
