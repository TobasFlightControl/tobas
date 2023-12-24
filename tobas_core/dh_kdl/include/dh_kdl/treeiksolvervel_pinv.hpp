#pragma once

#include "./treeiksolver.hpp"
#include "./treejnttojacsolver.hpp"

namespace KDL
{
class TreeIkSolverVel_pinv : public TreeIkSolverVel
{
  using super = TreeIkSolverVel;

public:
  explicit TreeIkSolverVel_pinv(const Tree& tree);

  void updateInternalDataStructures() override;

  int CartToJnt(const JntArray& q_in, const TwistMap& v_in) override;

  const Eigen::Vector6d& getWeightTS() const;
  bool setWeightTS(const Eigen::Vector6d& Wt);

  const double& getWeightJS() const;
  bool setWeightJS(const double& Wj);

private:
  TreeJntToJacSolver jnt2jac_;

  Eigen::Vector6d Wt_ = Eigen::Vector6d::Constant(kDefaultWeightTS);
  double Wj_ = kDefaultWeightJS;
};
}  // namespace KDL
