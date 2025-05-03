#include <iostream>

#include <tobas_eigen_tools/core.hpp>

#include "../include/tobas_control/lqid.hpp"
#include "../include/tobas_control/care.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
LQID::LQID(const Index& state_size, const Index& input_size, const Index& integrate_size)
  : dynamics(state_size, input_size)
  , C(integrate_size, state_size)
  , state_weight(state_size)
  , integrated_error_weight(integrate_size)
  , input_weight(input_size)
  , input_rate_weight(input_size)
  , current_state(state_size)
  , target_state(state_size)
  , max_integrated_error(integrate_size)
  ,

  x_size_(state_size)
  , u_size_(input_size)
  , r_size_(integrate_size)
  , x_tilde_size_(state_size + input_size + integrate_size)
  , x_idx_(0)
  , u_idx_(x_idx_ + x_size_)
  , eps_idx_(u_idx_ + u_size_)
  ,

  last_u_(u_size_)
  , eps_(r_size_)
  , x_tilde_(x_tilde_size_)
  , s_tilde_(x_tilde_size_)
  , A_tilde_(x_tilde_size_, x_tilde_size_)
  , B_tilde_(x_tilde_size_, u_size_)
  , Q_tilde_(x_tilde_size_, x_tilde_size_)
  , R_tilde_(u_size_, u_size_)
{
  C.setZero();
  state_weight.setZero();
  integrated_error_weight.setZero();
  input_weight.setZero();
  input_rate_weight.setZero();
  current_state.setZero();
  target_state.setZero();
  max_integrated_error.setZero();

  last_u_.setZero();
  eps_.setZero();
  x_tilde_.setZero();
  s_tilde_.setZero();
  A_tilde_.setZero();
  B_tilde_.setZero();
  Q_tilde_.setZero();
  R_tilde_.setZero();
}

VectorXd LQID::solve(const double& dt, const bool& update_gain)
{
  assert(dt >= 0);

  assert(current_state.rows() == x_size_);
  assert(eigen::isFinite(current_state));

  assert(target_state.rows() == x_size_);
  assert(eigen::isFinite(target_state));

  assert(max_integrated_error.rows() == r_size_);
  assert(eigen::isFinite(max_integrated_error));
  assert((max_integrated_error.array() >= 0).all());

  if (update_gain) {
    updateGain();
  }

  // 積分誤差を更新
  const VectorXd y = C * current_state;
  const VectorXd r = C * target_state;
  eps_ += (r - y) * dt;
  eps_ = eps_.cwiseMax(-max_integrated_error).cwiseMin(max_integrated_error);
  // cout << "Integrated error: " << eps_ << endl;

  // 拡大状態を作成
  x_tilde_ << current_state, last_u_, eps_;
  s_tilde_.block(0, 0, x_size_, 1) = target_state;

  // 最適制御入力変化率を計算
  const auto ud_scaled = K_ * (s_tilde_ - x_tilde_);

  // 最新の制御入力を更新
  last_u_ += ud_scaled * dt;

  return last_u_;
}

void LQID::updateGain()
{
  assert(dynamics.stateSize() == x_size_ && dynamics.inputSize() == u_size_);
  assert(dynamics.isFinite());

  assert(C.rows() == r_size_ && C.cols() == x_size_);
  assert(eigen::isFinite(C));

  assert(state_weight.rows() == x_size_);
  assert(eigen::isFinite(state_weight));
  assert((state_weight.array() >= 0).all());

  assert(integrated_error_weight.rows() == r_size_);
  assert(eigen::isFinite(integrated_error_weight));
  assert((integrated_error_weight.array() >= 0).all());

  assert(input_weight.rows() == u_size_);
  assert(eigen::isFinite(input_weight));
  assert((input_weight.array() >= 0).all());

  assert(input_rate_weight.rows() == u_size_);
  assert(eigen::isFinite(input_rate_weight));
  assert((input_rate_weight.array() > 0).all());

  // 拡大状態のダイナミクスを更新
  A_tilde_.block(x_idx_, x_idx_, x_size_, x_size_) = dynamics.A;
  A_tilde_.block(x_idx_, u_idx_, x_size_, u_size_) = dynamics.B;
  A_tilde_.block(eps_idx_, x_idx_, r_size_, x_size_) = -C;
  B_tilde_.block(u_idx_, 0, u_size_, u_size_).diagonal().setOnes();

  // 重み行列を更新
  Q_tilde_.block(x_idx_, x_idx_, x_size_, x_size_).diagonal() = state_weight;
  Q_tilde_.block(u_idx_, u_idx_, u_size_, u_size_).diagonal() = input_weight;
  Q_tilde_.block(eps_idx_, eps_idx_, r_size_, r_size_).diagonal() = integrated_error_weight;
  R_tilde_.diagonal() = input_rate_weight;

  // CAREを解く
  P_inf_ = care_ArimotoPotter(A_tilde_, B_tilde_, Q_tilde_, R_tilde_);

  // LQRの解を計算
  K_ = R_tilde_.diagonal().cwiseInverse().asDiagonal() * B_tilde_.transpose() * P_inf_;
}

ostream& operator<<(ostream& os, const LQID& arg)
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
