#pragma once

#include <unordered_map>

#include "./treesolveri.hpp"
#include "./jntarray.hpp"

namespace kdl
{
class TreeJointParser : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJointParser(const Tree& tree);

  void updateInternalDataStructures() override;

  inline const std::vector<std::string>& jointNames() const;
  inline const std::string& jointName(const size_t& q_nr) const;

  inline const std::unordered_map<std::string, size_t>& jointIndexMap() const;
  inline const size_t& jointIndex(const std::string& jnt_name) const;

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

  /* 関節がTree内に存在するかどうかを返す． */
  inline bool exist(const std::string& jnt_name) const;

private:
  std::vector<std::string> jnt_names_;
  std::unordered_map<std::string, size_t> jnt_idx_;
  JntArray lower_limits_;
  JntArray upper_limits_;
  JntArray max_velocities_;
  JntArray max_efforts_;

  /* 全ての駆動関節名を取得し，RNEと同じ順番に並べる． */
  void parseJntNames();

  void parseJntNamesStep(const SegmentMap::const_iterator& segment);
};

inline const std::vector<std::string>& TreeJointParser::jointNames() const
{
  return jnt_names_;
}

inline const std::string& TreeJointParser::jointName(const size_t& q_nr) const
{
  return jnt_names_.at(q_nr);
}

inline const std::unordered_map<std::string, size_t>& TreeJointParser::jointIndexMap() const
{
  return jnt_idx_;
}

inline const size_t& TreeJointParser::jointIndex(const std::string& jnt_name) const
{
  return jnt_idx_.at(jnt_name);
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
  return jnt_idx_.contains(jnt_name);
}
}  // namespace kdl
