#include <tobas_std_tools/vector.hpp>

#include "../include/tobas_kdl/treejointstateconverter.hpp"

using namespace std;

namespace kdl
{
TreeJointStateConverter::TreeJointStateConverter(const Tree& tree) : super(tree), jnt_parser_(tree_)
{
  updateInternalDataStructures();
}

void TreeJointStateConverter::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jnt_parser_.updateInternalDataStructures();

  q_out_.resize(nj_);
  qd_out_.resize(nj_);
  f_out_.resize(nj_);
}

int TreeJointStateConverter::jointStateToJntArrayPos(const sensor_msgs::msg::JointState& js)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (js.position.size() != js.name.size())
    return setDefaultError(E_SIZE_MISMATCH);

  for (size_t msg_idx = 0; msg_idx < js.name.size(); ++msg_idx)
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(js.name[msg_idx]);
      q_out_(kdl_idx) = js.position.at(msg_idx);
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jointStateToJntArrayVel(const sensor_msgs::msg::JointState& js)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (js.velocity.size() != js.name.size())
    return setDefaultError(E_SIZE_MISMATCH);

  for (size_t msg_idx = 0; msg_idx < js.name.size(); ++msg_idx)
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(js.name[msg_idx]);
      qd_out_(kdl_idx) = js.velocity.at(msg_idx);
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jointStateToJntArrayEff(const sensor_msgs::msg::JointState& js)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (js.effort.size() != js.name.size())
    return setDefaultError(E_SIZE_MISMATCH);

  for (size_t msg_idx = 0; msg_idx < js.name.size(); ++msg_idx)
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(js.name[msg_idx]);
      f_out_(kdl_idx) = js.effort.at(msg_idx);
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jointStateToJntArrayPosVel(const sensor_msgs::msg::JointState& js)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);

  const auto size = js.name.size();
  if (js.position.size() != size || js.velocity.size() != size)
    return setDefaultError(E_SIZE_MISMATCH);

  for (size_t msg_idx = 0; msg_idx < size; ++msg_idx)
  {
    const auto& jnt_name = js.name[msg_idx];
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
      q_out_(kdl_idx) = js.position.at(msg_idx);
      qd_out_(kdl_idx) = js.velocity.at(msg_idx);
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jointStateToJntArray(const sensor_msgs::msg::JointState& js)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);

  const auto size = js.name.size();
  if (js.position.size() != size || js.velocity.size() != size || js.effort.size() != size)
    return setDefaultError(E_SIZE_MISMATCH);

  for (size_t msg_idx = 0; msg_idx < size; ++msg_idx)
  {
    const auto& jnt_name = js.name[msg_idx];
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);  // Tree内でのインデックス
      q_out_(kdl_idx) = js.position.at(msg_idx);
      qd_out_(kdl_idx) = js.velocity.at(msg_idx);
      f_out_(kdl_idx) = js.effort.at(msg_idx);
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jntArrayToJointStatePos(const JntArray& q, const std::vector<std::string>& jnt_names)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  clearJointState();

  for (const auto& jnt_name : jnt_names)
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);
      js_out_.name.push_back(jnt_name);
      js_out_.position.push_back(q(kdl_idx));
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jntArrayToJointStateVel(const JntArray& qd, const std::vector<std::string>& jnt_names)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  clearJointState();

  for (const auto& jnt_name : jnt_names)
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);
      js_out_.name.push_back(jnt_name);
      js_out_.velocity.push_back(qd(kdl_idx));
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jntArrayToJointStateEff(const JntArray& f, const std::vector<std::string>& jnt_names)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (f.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  clearJointState();

  for (const auto& jnt_name : jnt_names)
  {
    try
    {
      const auto& kdl_idx = jnt_parser_.jointIndex(jnt_name);
      js_out_.name.push_back(jnt_name);
      js_out_.effort.push_back(f(kdl_idx));
    }
    catch (const exception& e)
    {
      error_msg_ = e.what();
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

int TreeJointStateConverter::jntArrayToJointState(
  const JntArray& q,
  const JntArray& qd,
  const JntArray& f,
  const vector<string>& jnt_names)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_ || f.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  clearJointState();

  for (const auto& jnt_name : jnt_names)
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
      return (error_code_ = E_UNKNOWN);
    }
  }

  return setDefaultError(E_NOERROR);
}

void TreeJointStateConverter::clearJointState()
{
  js_out_.name.clear();
  js_out_.position.clear();
  js_out_.velocity.clear();
  js_out_.effort.clear();
}
}  // namespace kdl
