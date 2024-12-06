#pragma once

#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_kdl/tree_solver_i.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>

namespace tobas
{
/* sensor_msgs::msg::JointStateとKDL::JntArrayの変換． */
class TreeJointStateConverter : public kdl::TreeSolverI
{
  using super = kdl::TreeSolverI;

public:
  static constexpr int E_JOINT_NAMES_NOT_SET = -100;

  explicit TreeJointStateConverter(const kdl::Tree& tree);

  void updateInternalDataStructures() override;

  int jointStateToJntArrayPos(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArrayVel(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArrayEff(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArrayPosVel(const sensor_msgs::msg::JointState& js);
  int jointStateToJntArray(const sensor_msgs::msg::JointState& js);

  int jntArrayToJointStatePos(const kdl::JntArray& q, const std::vector<std::string>& jnt_names);
  int jntArrayToJointStateVel(const kdl::JntArray& qd, const std::vector<std::string>& jnt_names);
  int jntArrayToJointStateEff(const kdl::JntArray& f, const std::vector<std::string>& jnt_names);
  int jntArrayToJointState(
    const kdl::JntArray& q,
    const kdl::JntArray& qd,
    const kdl::JntArray& f,
    const std::vector<std::string>& jnt_names);

  inline const kdl::JntArray& getPositionsKDL() const;
  inline const kdl::JntArray& getVelocitiesKDL() const;
  inline const kdl::JntArray& getEffortsKDL() const;

  inline const sensor_msgs::msg::JointState& getJointState() const;
  inline const std::vector<std::string>& getNamesMsg() const;
  inline const std::vector<double>& getPositionsMsg() const;
  inline const std::vector<double>& getVelocitiesMsg() const;
  inline const std::vector<double>& getEffortsMsg() const;

private:
  kdl::TreeJointParser jnt_parser_;

  kdl::JntArray q_out_;
  kdl::JntArray qd_out_;
  kdl::JntArray f_out_;
  sensor_msgs::msg::JointState js_out_;

  void clearJointState();
};

inline const kdl::JntArray& TreeJointStateConverter::getPositionsKDL() const
{
  return q_out_;
}

inline const kdl::JntArray& TreeJointStateConverter::getVelocitiesKDL() const
{
  return qd_out_;
}

inline const kdl::JntArray& TreeJointStateConverter::getEffortsKDL() const
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
}  // namespace tobas
