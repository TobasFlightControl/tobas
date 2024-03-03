#pragma once

#include "./qpsolver.hpp"

namespace quadprog
{
/**
 * @brief A numerically stable dual method for solving strictly convex quadratic programs
 * [D. Goldfarb & A. Idnani]
 */
class DualActiveSetSolver : public QuadProgSolver
{
  using super = QuadProgSolver;

public:
  explicit DualActiveSetSolver();

  Eigen::VectorXd solve() override;

private:
  enum state_t
  {
    CHOOSE_VIOLATED_CONSTRAINT,
    CHECK_FEASIBILITY,
    DETERMINE_STEP_DIRECTION,
  };

  size_t n_;
  size_t p_;  // The number of equality constraints
  size_t m_;  // The number of inequality constraints
  size_t l_;
  size_t ip_;  // The index of the constraint to be added to the active set
  size_t iq_;
  double c_, ss_, R_norm_;
  Eigen::MatrixXd R_, J_;
  Eigen::VectorXd s_, z_, r_, d_, np_, x_, u_, x_old_, u_old_;
  Eigen::VectorXi A_, A_old_, iai_;
  std::vector<bool> iaexcl_;

  void update_r();
  bool addConstraint();
  void deleteConstraint(const size_t& l);

  // Euclidean distance between two numbers.
  static double distance(const double& a, const double& b);
};
}  // namespace quadprog
