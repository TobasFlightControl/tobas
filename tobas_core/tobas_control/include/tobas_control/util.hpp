#pragma once

#include <vector>

#include <eigen3/Eigen/Core>

#include "./equations.hpp"

namespace ctrl
{
/**
 * @brief 可制御性行列を作る．
 *
 * @param A,B 連続時間のダイナミクス
 *
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd ctrb(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

/**
 * @brief 可観測性行列を作る．
 *
 * @param A,C 連続時間のダイナミクス
 *
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd obsv(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C);

/**
 * @brief 可制御か否かを判定する．
 *
 * @param A,B 連続時間のダイナミクス
 *
 * @return bool
 */
bool isControllable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

/**
 * @brief 可観測か否かを判定する．
 *
 * @param A,C 連続時間のダイナミクス
 *
 * @return bool
 */
bool isObservable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C);

/**
 * @brief 変数ベクトルxの範囲(lb <= x <= ub)から等価な行列不等式(A @ x <= b)を作る．
 *
 * @param lb 下限
 * @param ub 上限
 * @param inf これ以上の値を行列不等式から省く
 *
 * @return LinearEquation A @ x <= b の(A, b)
 */
LinearEquation matIneqFromRange(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const double& inf = 1e+12);

/**
 * @brief 位置の追従誤差が指数関数的に減衰する場合の，時刻tにおける位置を計算する．
 *
 * @param x0 初期位置
 * @param xd 目標位置
 * @param tau 減衰時定数
 * @param t 時刻(= 初期時刻0からの経過時間)
 *
 * @return double 時刻tにおける位置
 */
double firstOrderPos(const double& x0, const double& xd, const double& tau, const double& t);

/**
 * @brief 速度の追従誤差が指数関数的に減衰する場合の，時刻tにおける位置を計算する． (memo: 1-47)
 *
 * @param x0 初期位置
 * @param v0 初期速度
 * @param vd 目標速度
 * @param tau 減衰時定数
 * @param t 時刻(= 初期時刻0からの経過時間)
 *
 * @return double 時刻tにおける位置
 */
double firstOrderVel(const double& x0, const double& v0, const double& vd, const double& tau, const double& t);
}  // namespace ctrl
