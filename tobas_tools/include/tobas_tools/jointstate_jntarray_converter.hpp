#pragma once

#include <sensor_msgs/JointState.h>

#include <dh_kdl/treejntparser.hpp>

#include "./solveri.hpp"
#include "./drone.hpp"

namespace tobas
{
class JointStateJntArrayConverter : public SolverI
{
public:
  explicit JointStateJntArrayConverter(const Drone& drone);

  void updateInternalDataStructures() override;

  int jointStateToJntArray(const sensor_msgs::JointState& js);
  int jntArrayToJointState(const KDL::JntArray& q, const KDL::JntArray& qd, const KDL::JntArray& f);

  inline const KDL::JntArray& getPositions() const;
  inline const KDL::JntArray& getVelocities() const;
  inline const KDL::JntArray& getEfforts() const;
  inline const sensor_msgs::JointState& getJointState() const;

private:
  const Drone& drone_;
  KDL::TreeJointParser jnt_parser_;
  uint32_t nj_;

  KDL::JntArray q_out_;
  KDL::JntArray qd_out_;
  KDL::JntArray f_out_;
  sensor_msgs::JointState js_out_;
};

inline const KDL::JntArray& JointStateJntArrayConverter::getPositions() const
{
  return q_out_;
}

inline const KDL::JntArray& JointStateJntArrayConverter::getVelocities() const
{
  return qd_out_;
}

inline const KDL::JntArray& JointStateJntArrayConverter::getEfforts() const
{
  return f_out_;
}

inline const sensor_msgs::JointState& JointStateJntArrayConverter::getJointState() const
{
  return js_out_;
}
}  // namespace tobas
