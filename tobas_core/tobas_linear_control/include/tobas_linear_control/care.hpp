#pragma once

#include <Eigen/Core>

namespace ctrl
{
/**
 * @brief 連続時間代数リッカチ方程式を解く．
 * cf. 有本/ポッター法: https://qiita.com/trgkpc/items/8210927d5b035912a153
 */
Eigen::MatrixXd care_ArimotoPotter(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& Q,
  const Eigen::MatrixXd& R);

/**
 * @brief 連続時間代数リッカチ方程式を解く．
 *
 * @note 正しく動作しない (2023/5/24)
 */
Eigen::MatrixXd
care_Schur(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
}  // namespace ctrl
