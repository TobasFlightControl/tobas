#include <dh_std_tools/vector.hpp>

#include "../include/tobas_tools/jointstate_jntarray_converter.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
explicit JointStateJntArrayConverter::JointStateJntArrayConverter(const Drone& drone)
  : drone_(drone), jnt_parser_(drone_.tree())
{
  updateInternalDataStructures();
}

void JointStateJntArrayConverter::updateInternalDataStructures()
{
  jnt_parser_.updateInternalDataStructures();

  nj_ = drone_.tree().getNrOfJoints();
  q_.resize(nj_);
}

const JntArray& JointStateJntArrayConverter::convert(const sensor_msgs::JointState& js)
{
  // FIXME: segfaultのリスクあり
  for (const auto& jnt_name : drone_.postureDefiningJoints())
  {
    const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);     // Tree内でのインデックス
    const auto msg_idx = dh_std::findIndex(js.name, jnt_name);  // msg内でのインデックス
    q_(kdl_idx) = js.position[msg_idx];
  }
  return q_;
}
}  // namespace tobas
