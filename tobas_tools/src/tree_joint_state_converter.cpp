#include <tobas_std_tools/vector.hpp>

#include "../include/tobas_tools/tree_joint_state_converter.hpp"

namespace tobas
{
TreeJointStateConverter::TreeJointStateConverter(const kdl::Tree& tree) : super(tree), jnt_parser_(tree_)
{
  resize();
  setZero();
}

bool TreeJointStateConverter::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!jnt_parser_.updateInternalDataStructures()) {
    return false;
  }

  resize();

  return true;
}

int TreeJointStateConverter::convert(const tobas_msgs::msg::JointStateArray& msg)
{
  if (!isUpToDate()) {
    return setDefaultError(E_NOT_UP_TO_DATE);
  }

  for (const auto& state : msg.states) {
    try {
      const auto& kdl_idx = jnt_parser_.jointIndex(state.name);  // Tree内でのインデックス
      q_out_(kdl_idx) = state.position;
      qd_out_(kdl_idx) = state.velocity;
      f_out_(kdl_idx) = state.effort;
    }
    catch (const std::exception& e) {
      error_msg_ = e.what();
      return error_code_ = E_NOERROR;
    }
  }

  return setDefaultError(E_NOERROR);
}

void TreeJointStateConverter::resize()
{
  q_out_.resize(nj_);
  qd_out_.resize(nj_);
  f_out_.resize(nj_);
}

void TreeJointStateConverter::setZero()
{
  q_out_.setZero();
  qd_out_.setZero();
  f_out_.setZero();
}
}  // namespace tobas
