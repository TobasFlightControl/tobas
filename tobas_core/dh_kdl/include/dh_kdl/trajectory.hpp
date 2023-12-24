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
    const KDL::Vector& p0,
    const KDL::Vector& pf,
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
  void get(const double& t, const KDL::Rotation& r, KDL::Vector& p, KDL::Vector& v, KDL::Vector& a);

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   * @param a 時刻tにおける加速度
   */
  void get(const double& t, KDL::Vector& p, KDL::Vector& v, KDL::Vector& a);

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   * @param v 時刻tにおける速度
   */
  void get(const double& t, KDL::Vector& p, KDL::Vector& v);

  /**
   * @brief 時刻tにおける軌跡を得る
   *
   * @param t 開始点からの時刻
   * @param p 時刻tにおける位置
   */
  void get(const double& t, KDL::Vector& p);

private:
  KDL::Vector p0_;
  KDL::Vector pf_;
  double T_;
  double h_;
  double k_;
  double TT_;
  double kk_;
  KDL::Vector p_diff_;
  KDL::Vector dummy_vector_;
  const KDL::Rotation r0_;  // 単位行列

  void getPos(const double& t, const KDL::Rotation& r, KDL::Vector& p);
  void getVel(const double& t, const KDL::Rotation& r, KDL::Vector& v);
  void getAcc(const double& t, const KDL::Rotation& r, KDL::Vector& a);
};
}  // namespace tj
