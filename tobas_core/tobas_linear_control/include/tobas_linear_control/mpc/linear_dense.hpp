#pragma once

#include <tobas_std_tools/stopwatch.hpp>

// #include <tobas_quadprog/quadprogpp.hpp>
// #include <tobas_quadprog/qpoases.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
// #include <tobas_quadprog/primal_dual_interior_point.hpp>

#include "../state_spaces.hpp"
#include "../equations.hpp"

namespace ctrl
{
/**
 * @brief 線形モデル予測制御．
 * cf. https://www.tdupress.jp/book/b349347.html
 */
class LinearDenseMPC
{
public:
  // Dynamics
  std::vector<LinearDynamics> discrete_dynamics;  // x[k+1] = A[k]x[k] + B[k]u[k]: 離散状態方程式
  Eigen::MatrixXd Cz;                             // z = Cz x: 制御変数方程式

  // Time horizon
  Eigen::Index prediction_steps;  // H_p: 状態の予測ステップ数
  Eigen::Index input_steps;       // H_u: 制御入力の予測ステップ数
  double time_step;               // dt: 離散化の間隔 [s]

  // tau: 制御変数の設定値と参照軌道との誤差の減衰時定数 [s]
  // 制御変数の振動が大きい場合，その時間微分の重みを上げるよりも参照軌道を調整する方が追従性能を落とさずに済むことが多い．
  // 参照軌道を時変にできるのがMPCの利点の1つでもある．
  Eigen::VectorXd decay_time_consts;

  // Scales
  Eigen::VectorXd state_scale;    // 状態変数のスケール
  Eigen::VectorXd input_scale;    // 制御入力のスケール
  Eigen::VectorXd control_scale;  // 制御変数のスケール

  // Weights
  Eigen::VectorXd input_rate_weight;  // R: 制御入力の変化率に対する重み (無次元)
  Eigen::VectorXd input_weight;       // S: 制御入力に対する重み (無次元)
  Eigen::VectorXd control_weight;     // Q: 制御変数に対する重み (無次元)

  // Equality constraints
  std::vector<LinearEquation> input_rate_eqs;  // Ee du <= ee: 制御入力の変化率に対する等式制約
  std::vector<LinearEquation> input_eqs;       // Fe u <= fe: 制御入力に対する等式制約
  std::vector<LinearEquation> control_eqs;     // Ge z <= ge: 制御変数に対する等式制約

  // Inequality constraints
  std::vector<LinearEquation> input_rate_ineqs;  // Ei du <= ei: 制御入力の変化率に対する不等式制約
  std::vector<LinearEquation> input_ineqs;       // Fi u <= fi: 制御入力に対する不等式制約
  std::vector<LinearEquation> control_ineqs;     // Gi z <= gi: 制御変数に対する不等式制約

  // States
  Eigen::VectorXd current_state;  // x: 現在の状態
  Eigen::VectorXd set_state;      // s: 設定値

  explicit LinearDenseMPC();

  void solve();

  inline const Eigen::VectorXd& optimalControlInput() const;

  friend std::ostream& operator<<(std::ostream& os, const LinearDenseMPC& arg);

private:
  // quadprog::QuadProgppSolver qpsolver_;
  // quadprog::QpOasesSolver qpsolver_;
  quadprog::DualActiveSetSolver qpsolver_;
  // quadprog::PrimalDualInteriorPointSolver qpsolver_;

  bool is_first_solve_ = true;
  Eigen::Index x_size_, u_size_, z_size_;
  Eigen::VectorXd last_input_;  // u: 最新の制御入力

  tobas_std::Stopwatch stopwatch_;

  void checkProblemValidity();

  void updateQpConstraint(
    const Eigen::VectorXd& last_u,
    const Eigen::VectorXd& Psi_x,
    const Eigen::VectorXd& Upsilon_u,
    const Eigen::MatrixXd& Theta,
    const std::vector<LinearEquation>& du_consts,
    const std::vector<LinearEquation>& u_consts,
    const std::vector<LinearEquation>& z_consts,
    Eigen::MatrixXd& A,
    Eigen::VectorXd& b);

  Eigen::MatrixXd makeSa();
  Eigen::VectorXd makeSb(const Eigen::VectorXd& last_u);
  Eigen::MatrixXd makeFGothic(const Eigen::MatrixXd& F);
  Eigen::MatrixXd makePsi(const std::vector<LinearDynamics>& dyn, const Eigen::MatrixXd& Cz);
  Eigen::MatrixXd makeUpsilon(const std::vector<LinearDynamics>& dyn, const Eigen::MatrixXd& Cz);
  Eigen::MatrixXd makeTheta(const std::vector<LinearDynamics>& dyn, const Eigen::MatrixXd& Cz);

  /* p.12の例題1.3を参考にp.90のTauを作成． */
  Eigen::VectorXd makeTau(const Eigen::VectorXd& x, const Eigen::VectorXd& z, const Eigen::MatrixXd& Cz);
  std::vector<Eigen::VectorXd> makeDecays();

  /* 不等式条件式 A x <= b の時系列を受け取り，全体の不等式成約行列を作る． */
  static Eigen::MatrixXd makeConstraintMatrix(const std::vector<LinearEquation>& ineqs, const Eigen::Index& H);
};

inline const Eigen::VectorXd& LinearDenseMPC::optimalControlInput() const
{
  return last_input_;
}
}  // namespace ctrl
