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

  explicit LQID(const size_t& state_size, const size_t& input_size, const size_t& integrate_size);

  Eigen::VectorXd solve(const double& dt, const bool& update_gain = true);
  void updateGain();

  inline const Eigen::VectorXd& integralError() const;

  friend std::ostream& operator<<(std::ostream& os, const LQID& arg);

private:
  const size_t x_size_;        // 状態のサイズ
  const size_t u_size_;        // 制御入力のサイズ
  const size_t r_size_;        // 積分する変数のサイズ
  const size_t x_tilde_size_;  // 拡大状態のサイズ
  const size_t x_idx_;
  const size_t u_idx_;
  const size_t eps_idx_;

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

inline const Eigen::VectorXd& LQID::integralError() const
{
  return eps_;
}
}  // namespace ctrl
