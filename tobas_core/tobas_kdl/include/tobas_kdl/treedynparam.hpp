#pragma once

#include "./treesolveri.hpp"
#include "./jntspaceinertiamatrix.hpp"
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
   * @brief 単位ベクトル法により関節空間の慣性行列を計算する
   *
   * @param q 関節角
   * @param qd 関節角速度
   *
   * @note Hはqにのみ依存するため，実際のところqdは不要
   */
  int JntToMass(const JntArray& q, const JntArray& qd);

  /**
   * @brief 単位ベクトル法により関節空間の慣性行列を計算する
   *
   * @param q 関節角
   */
  int JntToMass(const JntArray& q);

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

  inline const JntSpaceInertiaMatrix& getJntInertia() const;
  inline const JntArray& getCoriolisEffort() const;
  inline const JntArray& getGravityEffort() const;

private:
  TreeIdSolver_RNE rne_mass_;
  TreeIdSolver_RNE rne_coriolis_;
  TreeIdSolver_RNE rne_gravity_;

  const Vector vector_null_ = Vector::Zero();
  const WrenchMap wrenchmap_null_;
  JntArray jntarray_null_;
  std::vector<JntArray> elements_;

  JntSpaceInertiaMatrix H_out_;
};

inline const JntSpaceInertiaMatrix& TreeDynParam::getJntInertia() const
{
  return H_out_;
}

inline const JntArray& TreeDynParam::getCoriolisEffort() const
{
  return rne_coriolis_.getEfforts();
}

inline const JntArray& TreeDynParam::getGravityEffort() const
{
  return rne_gravity_.getEfforts();
}
}  // namespace KDL
