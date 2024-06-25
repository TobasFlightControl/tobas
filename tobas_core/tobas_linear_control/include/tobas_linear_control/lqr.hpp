#pragma once

#include "./state_spaces.hpp"

namespace ctrl
{
/**
 * @brief 無限時間区間の最適レギュレータ
 * cf. 現代制御論, 8.3節
 */
class LQR
{
public:
  LinearDynamics dynamics;  // xd = Ax + Bu: 連続時間状態方程式

  Eigen::VectorXd state_scale;  // 状態変数のスケール
  Eigen::VectorXd input_scale;  // 制御入力のスケール

  Eigen::VectorXd state_weight;  // Q: 状態変数に対する重み (無次元)
  Eigen::VectorXd input_weight;  // R: 制御入力に対する重み (無次元)

  Eigen::VectorXd current_state;  // x: 現在の状態
  Eigen::VectorXd target_state;   // s: 設定値

  explicit LQR();

  Eigen::VectorXd solve(const bool& update_gain = true);
  void resize(const Eigen::Index& state_size, const Eigen::Index& input_size);
  void updateGain();

  friend std::ostream& operator<<(std::ostream& os, const LQR& arg);

private:
  Eigen::MatrixXd P_inf_;
  Eigen::MatrixXd K_;

  void checkProblemValidity();
};
}  // namespace ctrl
