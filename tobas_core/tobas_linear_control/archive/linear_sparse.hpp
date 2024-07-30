#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

#include "../state_spaces.hpp"
#include "../equations.hpp"
#include "../quadprog/core.hpp"

namespace ctrl
{
class LinearSparseMPC
{
public:
  explicit LinearSparseMPC(
    const std::vector<LinearDynamics>& disc_dyns,
    const Eigen::MatrixXd& Cz,
    const int& Hp,
    const double& dt,
    const std::vector<double>& decay_time_consts,
    const Eigen::VectorXd& R,
    const Eigen::VectorXd& S,
    const Eigen::VectorXd& Q,
    const LinearEquation& E_e,
    const LinearEquation& F_f,
    const LinearEquation& G_g);

  Eigen::VectorXd step(const Eigen::VectorXd& x, const Eigen::VectorXd& s);

  Eigen::VectorXd
  step(const Eigen::VectorXd& x, const Eigen::VectorXd& s, const std::vector<LinearDynamics>& disc_dyns);

private:
  const int Hp_;
  const int x_size_;
  const int u_size_;
  const int z_size_;
  const int var_size_;
  const int eq_size_;
  const int ineq_size_;
  const Eigen::MatrixXd Cz_;
  const Eigen::MatrixXd minus_CQ_;
  const std::vector<Eigen::VectorXd> decays_;
  Eigen::MatrixXd A0_;
  Eigen::VectorXd dU_;
  Eigen::VectorXd last_u_;
  Eigen::MatrixXd G_;
  Eigen::MatrixXd CE_;
  Eigen::MatrixXd CI_;
  Eigen::VectorXd g0_;
  Eigen::VectorXd ce0_;
  Eigen::VectorXd ci0_;
  Eigen::VectorXd x_;
  ctrl::QuadProgSolver qpsolver_;

  std::vector<Eigen::VectorXd> makeDecays(double dt, const std::vector<double>& decay_time_consts);

  Eigen::MatrixXd
  makeG(const Eigen::VectorXd& R, const Eigen::VectorXd& S, const Eigen::VectorXd& Q, const Eigen::MatrixXd& Cz);

  Eigen::MatrixXd makeBaseCE();

  Eigen::MatrixXd
  makeCI(const Eigen::MatrixXd& E, const Eigen::MatrixXd& F, const Eigen::MatrixXd& G, const Eigen::MatrixXd& Cz);

  Eigen::VectorXd makeCi0(const Eigen::VectorXd& e, const Eigen::VectorXd& f, const Eigen::VectorXd& g);

  void updateDynamics(const std::vector<LinearDynamics>& disc_dyns);

  void updateCE(const std::vector<LinearDynamics>& disc_dyns);

  void updateG0(const Eigen::VectorXd& x, const Eigen::VectorXd& s);

  void updateCe0(const Eigen::VectorXd& x);
};
}  // namespace ctrl
