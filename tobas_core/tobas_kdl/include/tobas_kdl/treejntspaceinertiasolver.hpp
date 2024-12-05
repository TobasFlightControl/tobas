#pragma once

#include "./treesolveri.hpp"
#include "./jntspaceinertiamatrix.hpp"
#include "./treeidsolver_rne.hpp"

namespace kdl
{
/**
 * @brief kdl::ChainDynParamのTree版
 */
class TreeJntSpaceInertiaSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJntSpaceInertiaSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  /**
   * @brief 単位ベクトル法により関節空間の慣性行列を計算する
   *
   * @param q 関節角
   */
  int JntToMass(const JntArray& q);

  inline const JntSpaceInertiaMatrix& getMass() const;

private:
  TreeIdSolver_RNE rne_;

  std::vector<JntArray> elements_;
  JntSpaceInertiaMatrix H_out_;
  JntArray jntarray_null_;
};

inline const JntSpaceInertiaMatrix& TreeJntSpaceInertiaSolver::getMass() const
{
  return H_out_;
}
}  // namespace kdl
