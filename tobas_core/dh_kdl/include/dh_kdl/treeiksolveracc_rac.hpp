#pragma once

#include "./treeiksolver.hpp"
#include "./treejnttojacsolver.hpp"
#include "./treejnttojacaccsolver.hpp"

namespace KDL
{
class TreeIkSolverAcc_RAC : public TreeIkSolverAcc
{
  using super = TreeIkSolverAcc;

public:
  explicit TreeIkSolverAcc_RAC(const Tree& tree);

  void updateInternalDataStructures() override;

  int CartToJnt(const JntArray& q_in, const JntArray& qd_in, const AccelMap& acc_in) override;

  const Eigen::Vector6d& getWeightTS() const;
  bool setWeightTS(const Eigen::Vector6d& Wt);

  const double& getWeightJS() const;
  bool setWeightJS(const double& Wj);

private:
  TreeJntToJacSolver jnt2jac_;
  TreeJntToJacAccSolver jnt2jdqd_;

  Eigen::Vector6d Wt_ = Eigen::Vector6d::Constant(kDefaultWeightTS);  // Task space weight
  double Wj_ = kDefaultWeightJS;                                      // Joint space weight
};
}  // namespace KDL
