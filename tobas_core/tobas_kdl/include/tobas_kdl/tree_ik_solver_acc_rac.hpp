#pragma once

#include <tobas_quadprog/dual_active_set.hpp>

#include "./tree_ik_solver.hpp"
#include "./tree_jac_acc_solver.hpp"
#include "./tree_jacobian_solver.hpp"
#include "./tree_joint_parser.hpp"

namespace kdl
{
class TreeIkSolverAcc_RAC : public TreeIkSolverAcc
{
  using super = TreeIkSolverAcc;

public:
  explicit TreeIkSolverAcc_RAC(const Tree& tree);

  bool updateInternalDataStructures() override;

  int CartToJnt(const JntArray& q_in, const JntArray& qd_in, const AccelMap& acc_in) override;

  const Eigen::Vector6d& getWeightTS() const;
  bool setWeightTS(const Eigen::Vector6d& Wt);

  const double& getWeightJS() const;
  bool setWeightJS(const double& Wj);

private:
  TreeJacobianSolver jnt2jac_;
  TreeJacAccSolver jnt2jdqd_;
  TreeJointParser jntparser_;

  Eigen::Vector6d Wt_ = Eigen::Vector6d::Constant(kDefaultWeightTS);  // Task space weight
  double Wj_ = kDefaultWeightJS;                                      // Joint space weight
  Eigen::VectorXd qdd_min_, qdd_max_;                                 // Joint acceleration limits
  Eigen::MatrixXd J_;                                                 // Big jacobian
  Eigen::VectorXd a_;                                                 // Big acceleration in TS

  quadprog::DualActiveSetSolver qp_solver_;

  void resize();
};
}  // namespace kdl
