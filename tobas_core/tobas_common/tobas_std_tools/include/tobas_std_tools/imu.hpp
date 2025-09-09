#pragma once

namespace tobas_std
{
/**
 * @brief 加速度と地磁気から姿勢を求める．
 * cf. https://qiita.com/take4eng/items/da4d5dbbcd5a3fdfded8
 *
 * @param ax,ay,az 加速度センサの読み
 * @param mx,my,mz 地磁気センサの読み
 * @param mx_ref,my_ref,mz_ref 原点の地磁気
 * @param roll,pitch,yaw 姿勢 (出力)
 *
 * @note World, Local共にNWU座標系を想定．
 */
void eulerFromAccelMag(
  const double& ax,
  const double& ay,
  const double& az,
  const double& mx,
  const double& my,
  const double& mz,
  const double& mx_ref,
  const double& my_ref,
  const double& mz_ref,
  double& roll,
  double& pitch,
  double& yaw);
}  // namespace tobas_std
