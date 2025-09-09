#pragma once

#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_kdl/tree_solver_i.hpp>

#include <tobas_msgs/msg/joint_state_array.hpp>

namespace tobas
{
/* tobas_msgs/JointStateArray -> kdl::JntArray */
class TreeJointStateConverter : public kdl::TreeSolverI
{
  using super = kdl::TreeSolverI;

public:
  explicit TreeJointStateConverter(const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  int convert(const tobas_msgs::msg::JointStateArray& msg);

  inline const kdl::JntArray& getPosition() const;
  inline const kdl::JntArray& getVelocity() const;
  inline const kdl::JntArray& getEffort() const;

private:
  kdl::TreeJointParser jnt_parser_;

  kdl::JntArray q_out_;
  kdl::JntArray qd_out_;
  kdl::JntArray f_out_;

  void resize();
  void setZero();
};

inline const kdl::JntArray& TreeJointStateConverter::getPosition() const
{
  return q_out_;
}

inline const kdl::JntArray& TreeJointStateConverter::getVelocity() const
{
  return qd_out_;
}

inline const kdl::JntArray& TreeJointStateConverter::getEffort() const
{
  return f_out_;
}
}  // namespace tobas
