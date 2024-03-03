#include <iostream>
#include <Eigen/LU>

#include <tobas_eigen_tools/core.hpp>

#include "../include/tobas_linear_control/lqr.hpp"
#include "../include/tobas_linear_control/care.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
LQR::LQR()
{
}

VectorXd LQR::solve(const bool& update_gain)
{
  checkProblemValidity();

  if (update_gain)
    updateGain();

  // スケーリング
  const VectorXd x_scaled = current_state.array() / state_scale.array();
  const VectorXd s_scaled = target_state.array() / state_scale.array();

  const auto u_scaled = K_ * (s_scaled - x_scaled);
  return u_scaled.cwiseProduct(input_scale);
}

void LQR::resize(const size_t& state_size, const size_t& input_size)
{
  dynamics.resize(state_size, input_size);

  state_scale = VectorXd::Zero(state_size);
  input_scale = VectorXd::Zero(input_size);

  state_weight = VectorXd::Zero(state_size);
  input_weight = VectorXd::Zero(input_size);

  current_state = VectorXd::Zero(state_size);
  target_state = VectorXd::Zero(state_size);
}

void LQR::updateGain()
{
  const auto dyn_scaled = dynamics.scale(state_scale, input_scale);
  P_inf_ = care_ArimotoPotter(
    dyn_scaled.A, dyn_scaled.B, state_weight.asDiagonal(), input_weight.asDiagonal());
  K_ = input_weight.asDiagonal().inverse() * dyn_scaled.B.transpose() * P_inf_;
}

void LQR::checkProblemValidity()
{
  const auto x_size = current_state.rows();
  const auto u_size = input_weight.rows();

  assert(x_size > 0);
  assert(u_size > 0);

  assert(dynamics.stateSize() == x_size && dynamics.inputSize() == u_size);
  assert(dynamics.isFinite());

  assert(state_scale.rows() == x_size);
  assert((state_scale.array() > 0.).all());
  assert(input_scale.rows() == u_size);
  assert((input_scale.array() > 0.).all());

  assert(state_weight.rows() == x_size);
  assert((state_weight.array() >= 0.).all());
  assert(input_weight.rows() == u_size);
  assert((input_weight.array() > 0.).all());

  assert(current_state.rows() == x_size);
  assert(target_state.rows() == x_size);
}

ostream& operator<<(ostream& os, const LQR& arg)
{
  os << "Dynamics:\n" << arg.dynamics << endl;
  os << "Current state:\n" << arg.current_state << endl;
  os << "Target state:\n" << arg.target_state << endl;
  os << "State error:\n" << arg.target_state - arg.current_state << endl;
  os << "Covariance matrix:\n" << arg.P_inf_ << endl;
  os << "Gain:\n" << arg.K_ << endl;

  return os;
}
}  // namespace ctrl
