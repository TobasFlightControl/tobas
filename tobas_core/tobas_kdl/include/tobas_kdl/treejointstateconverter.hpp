#pragma once

#include <sensor_msgs/msg/joint_state.hpp>

#include "./treesolveri.hpp"
#include "./treejntparser.hpp"

namespace kdl
{
/* sensor_msgs::msg::JointStateとKDL::JntArrayの変換． */
class TreeJointStateConverter : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr int E_JOINT_NAMES_NOT_SET = -100;

  explicit TreeJointStateConverter(const Tree& tree);

  void updateInternalDataStructures() override;

  int jointStateToJntArrayPos(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArrayVel(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArrayEff(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArrayPosVel(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArray(const sensor_msgs::msg::JointState& js);

  int jntArrayToJointStatePos(const JntArray& q, const std::vector<std::string>& jnt_names);
  int jntArrayToJointStateVel(const JntArray& qd, const std::vector<std::string>& jnt_names);
  int jntArrayToJointStateEff(const JntArray& f, const std::vector<std::string>& jnt_names);
  int jntArrayToJointState(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& f,
    const std::vector<std::string>& jnt_names);

  inline const JntArray& getPositionsKDL() const;
  inline const JntArray& getVelocitiesKDL() const;
  inline const JntArray& getEffortsKDL() const;

  inline const sensor_msgs::msg::JointState& getJointState() const;
  inline const std::vector<std::string>& getNamesMsg() const;
  inline const std::vector<double>& getPositionsMsg() const;
  inline const std::vector<double>& getVelocitiesMsg() const;
  inline const std::vector<double>& getEffortsMsg() const;

private:
  TreeJointParser jnt_parser_;

  JntArray q_out_;
  JntArray qd_out_;
  JntArray f_out_;
  sensor_msgs::msg::JointState js_out_;

  void clearJointState();
};

inline const JntArray& TreeJointStateConverter::getPositionsKDL() const
{
  return q_out_;
}

inline const JntArray& TreeJointStateConverter::getVelocitiesKDL() const
{
  return qd_out_;
}

inline const JntArray& TreeJointStateConverter::getEffortsKDL() const
{
  return f_out_;
}

inline const sensor_msgs::msg::JointState& TreeJointStateConverter::getJointState() const
{
  return js_out_;
}

inline const std::vector<std::string>& TreeJointStateConverter::getNamesMsg() const
{
  return js_out_.name;
}

inline const std::vector<double>& TreeJointStateConverter::getPositionsMsg() const
{
  return js_out_.position;
}

inline const std::vector<double>& TreeJointStateConverter::getVelocitiesMsg() const
{
  return js_out_.velocity;
}

inline const std::vector<double>& TreeJointStateConverter::getEffortsMsg() const
{
  return js_out_.effort;
}
}  // namespace kdl
