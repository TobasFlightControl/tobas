#include "tobas_kdl/chain.hpp"

#include <ranges>
#include <unordered_set>

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

bool Chain::isValid(string& error_msg) const
{
  unordered_set<string> seg_names, jnt_names;

  for (const auto& [idx, seg] : views::enumerate(segments)) {
    const auto& seg_name = seg.name();
    if (!seg_names.insert(seg_name).second) {
      error_msg = "Segment name \"" + seg_name + "\" is duplicated.";
      return false;
    }

    if (idx != 0) {
      const auto& jnt_name = seg.joint().name;
      if (!jnt_names.insert(jnt_name).second) {
        error_msg = "Joint name \"" + jnt_name + "\" is duplicated.";
        return false;
      }

      if (!seg.isValid(error_msg)) {
        return false;
      }
    }
  }

  return true;
}

void Chain::addSegment(const Segment& segment)
{
  segments.push_back(segment);
  ++ns_;
  if (segment.joint().type != Joint::kFixed) {
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
