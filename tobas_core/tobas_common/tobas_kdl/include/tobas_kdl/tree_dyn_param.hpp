#pragma once

#include "./tree_id_solver_rne.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief kdl::ChainDynParamのTree版
 */
class TreeDynParam : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeDynParam(const Tree& tree, const Vector& grav = Vector(0., 0., -st::kGravity));

  bool updateInternalDataStructures() override;

  /**
   * @brief コリオリ力により発生するトルクを計算する
   *
   * @param q 関節角
   * @param qd 関節角速度
   */
  int jntToCoriolis(const JntArray& q, const JntArray& qd);

  /**
   * @brief 重力により発生するトルクを計算する
   *
   * @param q 関節角
   */
  int jntToGravity(const JntArray& q);

  inline const JntArray& getCoriolisEffort() const;
  inline const JntArray& getGravityEffort() const;

private:
  TreeIdSolver_RNE rne_coriolis_;
  TreeIdSolver_RNE rne_gravity_;

  JntArray jntarray_null_;

  void resize();
};

inline const JntArray& TreeDynParam::getCoriolisEffort() const
{
  return rne_coriolis_.getEfforts();
}

inline const JntArray& TreeDynParam::getGravityEffort() const
{
  return rne_gravity_.getEfforts();
}
}  // namespace kdl
}  // namespace tobas
