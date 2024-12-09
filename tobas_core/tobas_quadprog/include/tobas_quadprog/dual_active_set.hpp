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

  bool solve() override;

  /* Get the lagrange multipliers of the equality constraints. */
  Eigen::VectorXd getLagrangeMultipliersEq() const;

  /* Get the lagrange multipliers of the inequality constraints. */
  Eigen::VectorXd getLagrangeMultipliersIneq() const;

private:
  enum state_t
  {
    CHOOSE_VIOLATED_CONSTRAINT,
    CHECK_FEASIBILITY,
    DETERMINE_STEP_DIRECTION,
  };

  Eigen::Index n_;   // The number of optimization variables
  Eigen::Index p_;   // The number of equality constraints
  Eigen::Index m_;   // The number of inequality constraints
  Eigen::Index ip_;  // The index of the inequality constraint to be added to the active set
  Eigen::Index iq_;  // The number of active constraints
  double c_;
  double ss_;
  double R_norm_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd s_;  // = b - A x. This must be positive.
  Eigen::VectorXd z_;
  Eigen::VectorXd r_;
  Eigen::VectorXd d_;
  Eigen::VectorXd np_;
  Eigen::VectorXd x_, x_old_;  // Optimization variables
  Eigen::VectorXd u_, u_old_;  // Lagrange multipliers
  Eigen::VectorXi A_, A_old_;  // The index of active constraints corresponding to the lagrange multipliers
  Eigen::VectorXi iai_;
  std::vector<bool> iaexcl_;

  void update_r();
  bool addConstraint();
  void deleteConstraint(const Eigen::Index& l);

  // Euclidean distance between two numbers.
  static double distance(const double& a, const double& b);
};
}  // namespace quadprog
