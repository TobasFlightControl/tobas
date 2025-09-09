#pragma once

#include "./state_spaces.hpp"

namespace ctrl
{
/**
 * @brief 線形二次積分制御 (ドローン工学入門: p.213)．
 * 与えられたダイナミクスの制御入力の変化率を入力とする．
 */
class LQID
{
public:
  LinearDynamics dynamics;  // xd = Ax + Bu: 連続時間状態方程式
  Eigen::MatrixXd C;        // y = C x: 積分する変数を抽出する行列

  Eigen::VectorXd state_weight;             // Q: 状態変数に対する重み
  Eigen::VectorXd integrated_error_weight;  // Qi: 積分誤差に対する重み
  Eigen::VectorXd input_weight;             // R: 制御入力に対する重み
  Eigen::VectorXd input_rate_weight;        // S: 制御入力の変化率に対する重み

  Eigen::VectorXd current_state;  // x: 現在の状態
  Eigen::VectorXd target_state;   // s: 設定値

  // 積分誤差の最大値
  // 定常外乱を想定している場合は理論的にはリミットは不要
  // 外乱が急に取り除かれる可能性があるなら安全な値に設定すべき
  Eigen::VectorXd max_integrated_error;

  explicit LQID(const Eigen::Index& state_size, const Eigen::Index& input_size, const Eigen::Index& integrate_size);

  Eigen::VectorXd solve(const double& dt, const bool& update_gain = true);
  void updateGain();

  inline const Eigen::VectorXd& getIntegralError() const;

  friend std::ostream& operator<<(std::ostream& os, const LQID& arg);

private:
  const Eigen::Index x_size_;        // 状態のサイズ
  const Eigen::Index u_size_;        // 制御入力のサイズ
  const Eigen::Index r_size_;        // 積分する変数のサイズ
  const Eigen::Index x_tilde_size_;  // 拡大状態のサイズ
  const Eigen::Index x_idx_;
  const Eigen::Index u_idx_;
  const Eigen::Index eps_idx_;

  Eigen::VectorXd last_u_;   // 最新の制御入力
  Eigen::VectorXd eps_;      // 誤差の積分
  Eigen::VectorXd x_tilde_;  // 拡大状態
  Eigen::VectorXd s_tilde_;  // 拡大目標状態
  Eigen::MatrixXd A_tilde_;
  Eigen::MatrixXd B_tilde_;
  Eigen::MatrixXd Q_tilde_;
  Eigen::MatrixXd R_tilde_;

  Eigen::MatrixXd P_inf_;
  Eigen::MatrixXd K_;
};

inline const Eigen::VectorXd& LQID::getIntegralError() const
{
  return eps_;
}
}  // namespace ctrl
