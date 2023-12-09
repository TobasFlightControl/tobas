#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/util.hpp>

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

  nj_ = drone_.tree().getNrOfJoints();

  q_out_ = JntArray::Zero(nj_);
  qd_out_ = JntArray::Zero(nj_);
  f_out_ = JntArray::Zero(nj_);
}

int JointStateJntArrayConverter::jointStateToJntArray(const sensor_msgs::JointState& js)
{
  if (drone_.tree().getNrOfJoints() != nj_)
  {
    error_msg_ = kErrorNotUpToDate;
    return -1;
  }

  for (const auto& jnt_name : drone_.postureDefiningJoints())
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);     // Tree内でのインデックス
      const auto msg_idx = dh_std::findIndex(js.name, jnt_name);  // msg内でのインデックス
      q_out_(kdl_idx) = js.position.at(msg_idx);
      qd_out_(kdl_idx) = js.velocity.at(msg_idx);
      f_out_(kdl_idx) = js.effort.at(msg_idx);
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return -1;
    }
  }

  return 0;
}

int JointStateJntArrayConverter::jntArrayToJointState(
  const KDL::JntArray& q,
  const KDL::JntArray& qd,
  const KDL::JntArray& f)
{
  if (drone_.tree().getNrOfJoints() != nj_)
  {
    error_msg_ = kErrorNotUpToDate;
    return -1;
  }

  if (q.rows() != nj_ || qd.rows() != nj_, f.rows() != nj_)
  {
    error_msg_ = kErrorSizeMismatch;
    return -1;
  }

  dh_ros::clear(js_out_);

  for (const auto& jnt_name : drone_.postureDefiningJoints())
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
      js_out_.name.push_back(jnt_name);
      js_out_.position.push_back(q(kdl_idx));
      js_out_.velocity.push_back(qd(kdl_idx));
      js_out_.effort.push_back(f(kdl_idx));
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
