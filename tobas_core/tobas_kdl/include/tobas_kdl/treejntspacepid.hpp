#pragma once

#include "./treeidsolver_rne.hpp"

namespace kdl
{
class TreeJntSpacePID : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr double kDefaultStiffness = 25.;
  static constexpr double kDefaultDamping = 10.;

  explicit TreeJntSpacePID(const Tree& tree, const Vector& grav = Vector(0, 0, -tobas_std::kGravity));

  void updateInternalDataStructures() override;

  int CartToJnt(
    const JntArray& cur_q,
    const JntArray& cur_qd,
    const JntArray& tar_q,
    const JntArray& tar_qd,
    const JntArray& qdd_ff);
  int CartToJnt(const JntArray& cur_q, const JntArray& cur_qd, const JntArray& tar_q, const JntArray& tar_qd);

  bool setStiffness(const double& kp);
  bool setDamping(const double& kd);

  inline const JntArray& getEfforts() const;

private:
  TreeIdSolver_RNE rne_;
  JntArray zeros_;

  double kp_ = kDefaultStiffness;
  double kd_ = kDefaultDamping;
};

inline const JntArray& TreeJntSpacePID::getEfforts() const
{
  return rne_.getEfforts();
}
}  // namespace kdl
