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

  int convert(const sensor_msgs::JointState& js, KDL::JntArray& q);

private:
  const Drone& drone_;
  KDL::TreeJointParser jnt_parser_;
};
}  // namespace tobas
