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

  inline const KDL::JntArray& getPositionsKDL() const;
  inline const KDL::JntArray& getVelocitiesKDL() const;
  inline const KDL::JntArray& getEffortsKDL() const;

  inline const sensor_msgs::JointState& getJointState() const;
  inline const std::vector<std::string>& getNamesMsg() const;
  inline const std::vector<double>& getPositionsMsg() const;
  inline const std::vector<double>& getVelocitiesMsg() const;
  inline const std::vector<double>& getEffortsMsg() const;

private:
  const Drone& drone_;
  KDL::TreeJointParser jnt_parser_;
  uint32_t nj_;

  KDL::JntArray q_out_;
  KDL::JntArray qd_out_;
  KDL::JntArray f_out_;
  sensor_msgs::JointState js_out_;
};

inline const KDL::JntArray& JointStateJntArrayConverter::getPositionsKDL() const
{
  return q_out_;
}

inline const KDL::JntArray& JointStateJntArrayConverter::getVelocitiesKDL() const
{
  return qd_out_;
}

inline const KDL::JntArray& JointStateJntArrayConverter::getEffortsKDL() const
{
  return f_out_;
}

inline const sensor_msgs::JointState& JointStateJntArrayConverter::getJointState() const
{
  return js_out_;
}

inline const std::vector<std::string>& JointStateJntArrayConverter::getNamesMsg() const
{
  return js_out_.name;
}

inline const std::vector<double>& JointStateJntArrayConverter::getPositionsMsg() const
{
  return js_out_.position;
}

inline const std::vector<double>& JointStateJntArrayConverter::getVelocitiesMsg() const
{
  return js_out_.velocity;
}

inline const std::vector<double>& JointStateJntArrayConverter::getEffortsMsg() const
{
  return js_out_.effort;
}
}  // namespace tobas
