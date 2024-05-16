#include <iostream>

#include <tobas_std_tools/console.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_eigen_tools/core.hpp>

#include "../../include/tobas_linear_control/mpc/linear_sparse.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace ctrl
{
LinearSparseMPC::LinearSparseMPC(
  const vector<LinearDynamics>& disc_dyns,
  const MatrixXd& Cz,
  const int& Hp,
  const double& dt,
  const vector<double>& decay_time_consts,
  const VectorXd& R,
  const VectorXd& S,
  const VectorXd& Q,
  const LinearEquation& E_e,
  const LinearEquation& F_f,
  const LinearEquation& G_g)
  : Hp_(Hp),
    x_size_(disc_dyns[0].stateSize()),
    u_size_(disc_dyns[0].inputSize()),
    z_size_(Cz.rows()),
    var_size_((u_size_ * 2 + x_size_) * Hp_),
    eq_size_((u_size_ + x_size_) * Hp_),
    ineq_size_((E_e.equationSize() + F_f.equationSize() + G_g.equationSize()) * Hp),
    Cz_(Cz),
    minus_CQ_(-Cz.transpose() * Q),
    decays_(makeDecays(dt, decay_time_consts)),
    A0_(MatrixXd::Zero(x_size_, x_size_)),
    dU_(VectorXd::Zero(var_size_)),
    last_u_(VectorXd::Zero(u_size_)),
    G_(makeG(R, S, Q, Cz)),
    CE_(makeBaseCE()),
    CI_(makeCI(E_e.A, F_f.A, G_g.A, Cz)),
    g0_(VectorXd::Zero(0., var_size_)),
    ce0_(VectorXd::Zero(0., eq_size_)),
    ci0_(makeCi0(E_e.b, F_f.b, G_g.b)),
    x_(VectorXd::Zero(0., var_size_)),
    qpsolver_(var_size_, eq_size_, ineq_size_)
{
  assert(x_size_ > 0);
  assert(u_size_ > 0);
  assert(z_size_ > 0);
  assert(Hp_ > 0);
  assert(dt > 0.);
  assert(Cz.rows() == z_size_ && Cz.cols() == x_size_);
  assert(R.rows() == u_size_);
  assert(S.rows() == u_size_);
  assert(Q.rows() == z_size_);
  assert(E_e.variableSize() == u_size_);
  assert(F_f.variableSize() == u_size_);
  assert(G_g.variableSize() == z_size_);

  updateDynamics(disc_dyns);
}

VectorXd LinearSparseMPC::step(const VectorXd& x, const VectorXd& s)
{
  assert(x.rows() == x_size_);
  assert(s.rows() == z_size_);

  updateG0(x, s);
  updateCe0(x);
  double f_value = qpsolver_.solveQp(G_, g0_, CE_, ce0_, CI_, ci0_, dU_);

  last_u_ += dU_.block(0, 0, u_size_, 1);
  return last_u_;
}

VectorXd
LinearSparseMPC::step(const VectorXd& x, const VectorXd& s, const vector<LinearDynamics>& disc_dyns)
{
  updateDynamics(disc_dyns);
  return step(x, s);
}

vector<VectorXd> LinearSparseMPC::makeDecays(double dt, const vector<double>& decay_time_consts)
{
  assert(dt > 0.);
  assert(decay_time_consts.size() == z_size_);
  assert(all_ge(decay_time_consts, 0.));

  vector<VectorXd> decays(Hp_, VectorXd::Zero(z_size_));
  double coin_time;
  double T_ref;

  for (int i = 0; i < Hp_; ++i)
  {
    coin_time = dt * static_cast<double>(i + 1);
    for (int j = 0; j < z_size_; ++j)
    {
      T_ref = decay_time_consts[j];
      if (T_ref > 0.)
      {
        decays[i](j) = exp(-coin_time / T_ref);
      }
    }
  }

  return decays;
}

MatrixXd
LinearSparseMPC::makeG(const VectorXd& R, const VectorXd& S, const VectorXd& Q, const MatrixXd& Cz)
{
  int size = u_size_ * 2 + x_size_;
  MatrixXd R_diag = R.asDiagonal();
  MatrixXd S_diag = S.asDiagonal();
  MatrixXd Q_diag = Q.asDiagonal();
  MatrixXd CQC = Cz.transpose() * Q_diag * Cz;

  MatrixXd G = MatrixXd::Zero(var_size_, var_size_);
  for (int i = 0; i < Hp_; ++i)
  {
    G.block(size * i, size * i, u_size_, u_size_) = S_diag;
    G.block(size * i + u_size_, size * i + u_size_, u_size_, u_size_) = R_diag;
    G.block(size * i + u_size_ * 2, size * i + u_size_ * 2, x_size_, x_size_) = CQC;
  }
  return G;
}

MatrixXd LinearSparseMPC::makeBaseCE()
{
  int r = u_size_ + x_size_;
  int c = u_size_ * 2 + x_size_;
  MatrixXd Iu = MatrixXd::Identity(u_size_, u_size_);
  MatrixXd Ix = MatrixXd::Identity(x_size_, x_size_);

  MatrixXd CE = MatrixXd::Zero(eq_size_, var_size_);
  for (int i = 0; i < Hp_; ++i)
  {
    if (i >= 1)
    {
      CE.block(r * i, c * i - c, u_size_, u_size_) = Iu;
    }
    CE.block(r * i, c * i, u_size_, u_size_) = -Iu;
    CE.block(r * i, c * i + u_size_, u_size_, u_size_) = Iu;
    CE.block(r * i + u_size_, c * i + u_size_ * 2, x_size_, x_size_) = -Ix;
  }
  return CE;
}

MatrixXd
LinearSparseMPC::makeCI(const MatrixXd& E, const MatrixXd& F, const MatrixXd& G, const MatrixXd& Cz)
{
  int re = E.rows();
  int rf = F.rows();
  int rg = G.rows();
  int r = re + rf + rg;
  int c = u_size_ * 2 + x_size_;
  MatrixXd GC = G * Cz;

  MatrixXd CI_part = MatrixXd::Zero(r, c);
  CI_part.block(0, 0, rf, u_size_) = F;
  CI_part.block(rf, u_size_, re, u_size_) = E;
  CI_part.block(rf + re, u_size_ * 2, rg, x_size_) = GC;

  return eigen_tools::blockDiag(CI_part, Hp_);
}

VectorXd LinearSparseMPC::makeCi0(const VectorXd& e, const VectorXd& f, const VectorXd& g)
{
  VectorXd ci0_part = eigen_tools::concat(e, f, g, 0);
  return eigen_tools::tile(ci0_part, Hp_, 0);
}

void LinearSparseMPC::updateDynamics(const vector<LinearDynamics>& disc_dyns)
{
  assert(disc_dyns.size() == Hp_);
  for (const auto& dyn : disc_dyns)
  {
    assert(dyn.stateSize() == x_size_ && dyn.inputSize() == u_size_);
  }

  updateCE(disc_dyns);
  A0_ = disc_dyns[0].A;
}

void LinearSparseMPC::updateCE(const vector<LinearDynamics>& disc_dyns)
{
  int r = u_size_ + x_size_;
  int c = u_size_ * 2 + x_size_;

  for (int i = 0; i < Hp_; ++i)
  {
    if (i >= 1)
    {
      CE_.block(r * i + u_size_, c * i - x_size_, x_size_, x_size_) = disc_dyns[i].A;
    }
    CE_.block(r * i + u_size_, c * i, x_size_, u_size_) = disc_dyns[i].B;
  }
}

void LinearSparseMPC::updateG0(const VectorXd& x, const VectorXd& s)
{
  int size = u_size_ * 2 + x_size_;
  VectorXd err = s - Cz_ * x;
  VectorXd ref;
  for (int i = 0; i < Hp_; ++i)
  {
    ref = s - decays_[i].cwiseProduct(err);
    g0_.block(size * i + u_size_ * 2, 0, z_size_, 1) = minus_CQ_;
  }
}

void LinearSparseMPC::updateCe0(const VectorXd& x)
{
  ce0_.block(0, 0, u_size_, 1) = last_u_;
  ce0_.block(u_size_, 0, x_size_, 1) = -A0_ * x;
}
}  // namespace ctrl
