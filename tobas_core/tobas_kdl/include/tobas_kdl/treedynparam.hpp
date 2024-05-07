#pragma once

#include "./treeidsolver_rne.hpp"
#include "./utilities/constants.hpp"

namespace KDL
{
/**
 * @brief KDL::ChainDynParamのTree版
 */
class TreeDynParam : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeDynParam(const Tree& tree, const Vector& grav = Vector(0., 0., -kDefaultGravity));

  void updateInternalDataStructures() override;

  /**
   * @brief コリオリ力により発生するトルクを計算する
   *
   * @param q 関節角
   * @param qd 関節角速度
   */
  int JntToCoriolis(const JntArray& q, const JntArray& qd);

  /**
   * @brief 重力により発生するトルクを計算する
   *
   * @param q 関節角
   */
  int JntToGravity(const JntArray& q);

  inline const JntArray& getCoriolisEffort() const;
  inline const JntArray& getGravityEffort() const;

private:
  TreeIdSolver_RNE rne_coriolis_;
  TreeIdSolver_RNE rne_gravity_;

  JntArray jntarray_null_;
};

inline const JntArray& TreeDynParam::getCoriolisEffort() const
{
  return rne_coriolis_.getEfforts();
}

inline const JntArray& TreeDynParam::getGravityEffort() const
{
  return rne_gravity_.getEfforts();
}
}  // namespace KDL
