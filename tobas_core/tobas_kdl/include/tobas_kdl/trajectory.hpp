#pragma once

#include "./frames.hpp"

namespace tj
{
/* 3次元サイクロイドを生成 */
class CycloidGenerator3d
{
public:
  explicit CycloidGenerator3d();

  void generate(
    const tobas_kdl::Vector& p0,
    const tobas_kdl::Vector& pf,
    const double& T,
    const double& h,
    const double& k = 5.);

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param r 見たいフレームから計画フレームへの回転
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   * @param a 時刻tにおける加速度
   */
  void
  get(const double& t, const tobas_kdl::Rotation& r, tobas_kdl::Vector& p, tobas_kdl::Vector& v, tobas_kdl::Vector& a)
    const;

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   * @param a 時刻tにおける加速度
   */
  void get(const double& t, tobas_kdl::Vector& p, tobas_kdl::Vector& v, tobas_kdl::Vector& a) const;

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   */
  void get(const double& t, tobas_kdl::Vector& p, tobas_kdl::Vector& v) const;

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   */
  void get(const double& t, tobas_kdl::Vector& p) const;

private:
  tobas_kdl::Vector p0_;
  tobas_kdl::Vector pf_;
  double T_;
  double h_;
  double k_;
  double TT_;
  double kk_;
  tobas_kdl::Vector p_diff_;
  const tobas_kdl::Rotation r0_;  // 単位行列

  void getPos(const double& t, const tobas_kdl::Rotation& r, tobas_kdl::Vector& p) const;
  void getVel(const double& t, const tobas_kdl::Rotation& r, tobas_kdl::Vector& v) const;
  void getAcc(const double& t, const tobas_kdl::Rotation& r, tobas_kdl::Vector& a) const;
};
}  // namespace tj
