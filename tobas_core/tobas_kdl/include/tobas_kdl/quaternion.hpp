#pragma once

#include "./frames.hpp"

namespace kdl
{
class Quaternion
{
public:
  double x, y, z, w;

  explicit Quaternion(const double& x, const double& y, const double& z, const double& w);
  explicit Quaternion(const Rotation& rot);
  explicit Quaternion();

  /* 単位クォータニオン． */
  static Quaternion Identity();

  /* 等価角軸ベクトルからクオータニオンを作成． */
  static Quaternion AngleAxis(const Vector& w);

  /* オイラー角からクオータニオンを作成． */
  static Quaternion RPY(const double& roll, const double& pitch, const double& yaw);

  /* クオータニオンをオイラー角に変換． */
  void getRPY(double& roll, double& pitch, double& yaw) const;

  /* 複素共役クォータニオン． */
  Quaternion complexConjugate() const;

  /* 逆クオータニオン． */
  Quaternion inverse() const;

  /* 要素の2乗和． */
  double squaredNorm() const;

  /* L2ノルム． */
  double norm() const;

  /* 正規化する． */
  Quaternion normalize() const;

  /* 正規クオータニオンであればtrueを返す． */
  bool isNormalized() const;

  /* クォータニオンの時間微分．角速度はローカルで定義されていることに注意． */
  Quaternion differential(const Vector& angvel) const;

  /* 全要素をスカラーで割る． */
  Quaternion operator/(const double& rhs) const;

  /* 2つの回転の合計． */
  Quaternion operator*(const Quaternion& rhs) const;

  /* 3次元ベクトルを回転させる． */
  Vector operator*(const Vector& v) const;

  friend std::ostream& operator<<(std::ostream& os, const Quaternion& arg);
};
}  // namespace kdl
