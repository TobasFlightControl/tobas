#include "../include/tobas_kdl/treejntparser.hpp"

using namespace std;

namespace kdl
{
TreeJointParser::TreeJointParser(const Tree& tree) : super(tree)
{
  updateInternalDataStructures();
}

void TreeJointParser::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jnt_names_.resize(nj_);
  jnt_idx_.clear();
  lower_limits_.resize(nj_);
  upper_limits_.resize(nj_);
  max_velocities_.resize(nj_);
  max_efforts_.resize(nj_);

  parseJntNames();
}

void TreeJointParser::parseJntNames()
{
  parseJntNamesStep(tree_.getRootSegment());
}

void TreeJointParser::parseJntNamesStep(const SegmentMap::const_iterator& segment)
{
  const auto& seg = segment->second.segment;
  const auto& jnt = seg.getJoint();
  const auto& q_nr = segment->second.q_nr;
  if (seg.getJoint().type != Joint::Fixed)
  {
    const auto& name = seg.getJoint().name;
    jnt_names_[q_nr] = name;
    jnt_idx_[name] = q_nr;
    lower_limits_(q_nr) = jnt.lower_limit;
    upper_limits_(q_nr) = jnt.upper_limit;
    max_velocities_(q_nr) = jnt.max_velocity;
    max_efforts_(q_nr) = jnt.max_effort;
  }

  for (const auto& child : segment->second.children)
    parseJntNamesStep(child);
}
}  // namespace kdl
