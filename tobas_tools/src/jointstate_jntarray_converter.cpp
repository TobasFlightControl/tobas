#include <dh_std_tools/vector.hpp>

#include "../include/tobas_tools/jointstate_jntarray_converter.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
JointStateJntArrayConverter::JointStateJntArrayConverter(const Drone& drone)
  : drone_(drone), jnt_parser_(drone_.tree())
{
  updateInternalDataStructures();
}

void JointStateJntArrayConverter::updateInternalDataStructures()
{
  jnt_parser_.updateInternalDataStructures();
}

int JointStateJntArrayConverter::convert(const sensor_msgs::JointState& js, JntArray& q)
{
  if (q.rows() != drone_.tree().getNrOfJoints())
  {
    error_msg_ = "Joint array size mismatch.";
    return -1;
  }
  for (const auto& jnt_name : drone_.postureDefiningJoints())
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);     // Tree内でのインデックス
      const auto msg_idx = dh_std::findIndex(js.name, jnt_name);  // msg内でのインデックス
      q(kdl_idx) = js.position[msg_idx];
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return -1;
    }
  }

  return 0;
}
}  // namespace tobas
