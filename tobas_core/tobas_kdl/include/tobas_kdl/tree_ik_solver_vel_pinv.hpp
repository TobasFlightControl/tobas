#pragma once

#include <tobas_quadprog/dual_active_set.hpp>

#include "./tree_ik_solver.hpp"
#include "./tree_jacobian_solver.hpp"
#include "./tree_joint_parser.hpp"

namespace kdl
{
class TreeIkSolverVel_pinv : public TreeIkSolverVel
{
  using super = TreeIkSolverVel;

public:
  explicit TreeIkSolverVel_pinv(const Tree& tree);

  bool updateInternalDataStructures() override;

  int CartToJnt(const JntArray& q_in, const TwistMap& v_in) override;

  const Eigen::Vector6d& getWeightTS() const;
  bool setWeightTS(const Eigen::Vector6d& Wt);

  const double& getWeightJS() const;
  bool setWeightJS(const double& Wj);

private:
  TreeJacobianSolver jnt2jac_;
  TreeJointParser jntparser_;

  Eigen::Vector6d Wt_ = Eigen::Vector6d::Constant(kDefaultWeightTS);
  double Wj_ = kDefaultWeightJS;
  Eigen::MatrixXd J_;  // Big jacobian
  Eigen::VectorXd t_;  // Big velocity in TS

  quadprog::DualActiveSetSolver qp_solver_;

  void resize();
};
}  // namespace kdl
