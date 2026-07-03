// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeJointParser : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJointParser(const Tree& tree);

  bool updateInternalDataStructures() override;

  inline const std::string& jointName(const size_t& q_nr) const;
  inline const size_t& jointIndex(const std::string& jnt_name) const;
  inline const std::string& segmentName(const std::string& jnt_name) const;

  inline const Joint& joint(const std::string& jnt_name) const;

  inline const JntArray& lowerLimits() const;
  inline double lowerLimit(const size_t& q_nr) const;
  inline double lowerLimit(const std::string& jnt_name) const;

  inline const JntArray& upperLimits() const;
  inline double upperLimit(const size_t& q_nr) const;
  inline double upperLimit(const std::string& jnt_name) const;

  inline const JntArray& maxVelocities() const;
  inline double maxVelocity(const size_t& q_nr) const;
  inline double maxVelocity(const std::string& jnt_name) const;

  inline const JntArray& maxEfforts() const;
  inline double maxEffort(const size_t& q_nr) const;
  inline double maxEffort(const std::string& jnt_name) const;

  /* Return whether the joint exists in the `Tree`. */
  inline bool exist(const std::string& jnt_name) const;

private:
  std::vector<std::string> jnt_names_;
  std::map<std::string, size_t> jnt_indexes_;
  std::map<std::string, std::string> seg_names_;

  JntArray lower_limits_;
  JntArray upper_limits_;
  JntArray max_velocities_;
  JntArray max_efforts_;

  void resize();

  /* Get all actuated joint names and order them the same way as RNE. */
  void parseJntNames();

  void parseJntNamesStep(const SegmentMap::const_iterator& seg_it);
};

inline const std::string& TreeJointParser::jointName(const size_t& q_nr) const
{
  return jnt_names_.at(q_nr);
}

inline const size_t& TreeJointParser::jointIndex(const std::string& jnt_name) const
{
  return jnt_indexes_.at(jnt_name);
}

inline const std::string& TreeJointParser::segmentName(const std::string& jnt_name) const
{
  return seg_names_.at(jnt_name);
}

inline const Joint& TreeJointParser::joint(const std::string& jnt_name) const
{
  return tree_.getSegment(segmentName(jnt_name))->second.segment.joint();
}

inline const JntArray& TreeJointParser::lowerLimits() const
{
  return lower_limits_;
}

inline double TreeJointParser::lowerLimit(const size_t& q_nr) const
{
  return lower_limits_(q_nr);
}

inline double TreeJointParser::lowerLimit(const std::string& jnt_name) const
{
  return lower_limits_(jointIndex(jnt_name));
}

inline const JntArray& TreeJointParser::upperLimits() const
{
  return upper_limits_;
}

inline double TreeJointParser::upperLimit(const size_t& q_nr) const
{
  assert(q_nr < nj_);
  return upper_limits_(q_nr);
}

inline double TreeJointParser::upperLimit(const std::string& jnt_name) const
{
  return upper_limits_(jointIndex(jnt_name));
}

inline const JntArray& TreeJointParser::maxVelocities() const
{
  return max_velocities_;
}

inline double TreeJointParser::maxVelocity(const size_t& q_nr) const
{
  assert(q_nr < nj_);
  return max_velocities_(q_nr);
}

inline double TreeJointParser::maxVelocity(const std::string& jnt_name) const
{
  return max_velocities_(jointIndex(jnt_name));
}

inline const JntArray& TreeJointParser::maxEfforts() const
{
  return max_efforts_;
}

inline double TreeJointParser::maxEffort(const size_t& q_nr) const
{
  return max_efforts_(q_nr);
}

inline double TreeJointParser::maxEffort(const std::string& jnt_name) const
{
  return max_efforts_(jointIndex(jnt_name));
}

inline bool TreeJointParser::exist(const std::string& jnt_name) const
{
  return jnt_indexes_.contains(jnt_name);
}
}  // namespace kdl
}  // namespace tobas
