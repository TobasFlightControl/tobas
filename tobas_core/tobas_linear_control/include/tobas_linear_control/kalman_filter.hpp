#pragma once

#include "./state_spaces.hpp"

namespace ctrl
{
/**
 * @brief 線形カルマンフィルタ．
 * cf. [線形カルマンフィルタの基礎, 足立修一](https://www.tdupress.jp/book/b349390.html)
 */
class KalmanFilter
{
public:
  LinearStateSpace ss;  // x(k+1) = A x(k) + B u(k), y(k) = C x(k): 離散時間状態方程式
  Eigen::MatrixXd Bv;   // プロセスノイズ行列
  Eigen::MatrixXd Q;    // プロセスノイズの共分散
  Eigen::MatrixXd R;    // 観測ノイズの共分散
  Eigen::VectorXd y;    // 観測
  Eigen::VectorXd u;    // 制御入力 (あれば)

  explicit KalmanFilter();
  explicit KalmanFilter(
    const Eigen::Index& x_size,
    const Eigen::Index& u_size,
    const Eigen::Index& y_size,
    const Eigen::Index& v_size);

  void resize(
    const Eigen::Index& x_size,
    const Eigen::Index& u_size,
    const Eigen::Index& y_size,
    const Eigen::Index& v_size);
  void setZero();
  void initialize(const Eigen::VectorXd& init_x, const Eigen::MatrixXd& init_P);
  void update();

  const Eigen::VectorXd& state() const;
  const Eigen::MatrixXd& covariance() const;

private:
  Eigen::Index x_size_, u_size_, y_size_, v_size_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
};

/* 白色ノイズを含む一定値の推定． */
class IdentityKalmanFilter
{
public:
  Eigen::MatrixXd Q;  // プロセスノイズの共分散
  Eigen::MatrixXd R;  // 観測ノイズの共分散
  Eigen::VectorXd y;  // 観測

  explicit IdentityKalmanFilter(const Eigen::Index& size = 0);

  void resize(const Eigen::Index& size);
  void setZero();
  void initialize(const Eigen::VectorXd& init_x, const Eigen::MatrixXd& init_P);
  void update();

  const Eigen::VectorXd& state() const;
  const Eigen::MatrixXd& covariance() const;

private:
  KalmanFilter kf_;
};
}  // namespace ctrl
