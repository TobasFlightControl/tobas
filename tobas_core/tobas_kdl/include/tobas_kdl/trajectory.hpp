#pragma once

#include "./frames.hpp"

namespace tj
{
/* 3次元サイクロイドを生成 */
class CycloidGenerator3d
{
public:
  explicit CycloidGenerator3d();

  void generate(const kdl::Vector& p0, const kdl::Vector& pf, const double& T, const double& h, const double& k = 5.);

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param r 見たいフレームから計画フレームへの回転
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   * @param a 時刻tにおける加速度
   */
  void get(const double& t, const kdl::Rotation& r, kdl::Vector& p, kdl::Vector& v, kdl::Vector& a) const;

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   * @param a 時刻tにおける加速度
   */
  void get(const double& t, kdl::Vector& p, kdl::Vector& v, kdl::Vector& a) const;

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   */
  void get(const double& t, kdl::Vector& p, kdl::Vector& v) const;

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   */
  void get(const double& t, kdl::Vector& p) const;

private:
  kdl::Vector p0_;
  kdl::Vector pf_;
  double T_;
  double h_;
  double k_;
  double TT_;
  double kk_;
  kdl::Vector p_diff_;
  const kdl::Rotation r0_;  // 単位行列

  void getPos(const double& t, const kdl::Rotation& r, kdl::Vector& p) const;
  void getVel(const double& t, const kdl::Rotation& r, kdl::Vector& v) const;
  void getAcc(const double& t, const kdl::Rotation& r, kdl::Vector& a) const;
};
}  // namespace tj
