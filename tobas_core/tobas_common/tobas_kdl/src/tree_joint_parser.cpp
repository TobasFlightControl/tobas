#include "tobas_kdl/tree_joint_parser.hpp"

using namespace std;

namespace kdl
{
TreeJointParser::TreeJointParser(const Tree& tree) : super(tree)
{
  resize();
  parseJntNames();
}

bool TreeJointParser::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  resize();
  parseJntNames();

  return true;
}

void TreeJointParser::resize()
{
  jnt_names_.resize(nj_);
  jnt_indexes_.clear();
  seg_names_.clear();
  lower_limits_.resize(nj_);
  upper_limits_.resize(nj_);
  max_velocities_.resize(nj_);
  max_efforts_.resize(nj_);
}

void TreeJointParser::parseJntNames()
{
  parseJntNamesStep(tree_.getRootSegment());
}

void TreeJointParser::parseJntNamesStep(const SegmentMap::const_iterator& seg_it)
{
  const auto& segment = seg_it->second.segment;
  const auto& joint = segment.joint();
  const auto& q_nr = seg_it->second.q_nr;

  if (joint.type != Joint::kFixed) {
    jnt_names_[q_nr] = joint.name;
    jnt_indexes_[joint.name] = q_nr;
    seg_names_[joint.name] = segment.name();
    lower_limits_(q_nr) = joint.lower_limit;
    upper_limits_(q_nr) = joint.upper_limit;
    max_velocities_(q_nr) = joint.max_velocity;
    max_efforts_(q_nr) = joint.max_effort;
  }

  for (const auto& child : seg_it->second.children) {
    parseJntNamesStep(child);
  }
}
}  // namespace kdl
