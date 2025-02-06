#pragma once

namespace tobas_std
{
/**
 * @brief ZYXオイラー角からクォータニオンを計算．
 * cf. https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
void quaternionFromEuler(
  const double& roll,
  const double& pitch,
  const double& yaw,
  double& x,
  double& y,
  double& z,
  double& w);

/**
 * @brief クォータニオンからZYXオイラー角を計算．
 * cf. https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
void eulerFromQuaternion(
  const double& x,
  const double& y,
  const double& z,
  const double& w,
  double& roll,
  double& pitch,
  double& yaw);

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

/**
 * @brief 加速度と地磁気から姿勢を求める．
 * cf. https://qiita.com/take4eng/items/da4d5dbbcd5a3fdfded8
 *
 * @param ax,ay,az 加速度センサの読み
 * @param mx,my,mz 地磁気センサの読み
 * @param mx_ref,my_ref,mz_ref 原点の地磁気
 * @param qx,qy,qz,qw 姿勢 (出力)
 *
 * @note World, Local共にNWU座標系を想定．
 */
void quaternionFromAccelMag(
  const double& ax,
  const double& ay,
  const double& az,
  const double& mx,
  const double& my,
  const double& mz,
  const double& mx_ref,
  const double& my_ref,
  const double& mz_ref,
  double& qx,
  double& qy,
  double& qz,
  double& qw);

/**
 * @brief 緯度，経度，高度を三次元直行座標に変換する．
 * cf. https://qiita.com/Toramin10/items/fa0c8e79aaadf84ddb25
 *
 * @param latitude 緯度[deg]
 * @param longitude 経度[deg]
 * @param altitude 高度[m]
 * @param x,y,z 直行座標[m](出力)
 */
void gnssToCartAbsolute(
  const double& latitude,
  const double& longitude,
  const double& altitude,
  double& x,
  double& y,
  double& z);

/**
 * @brief 緯度，経度を平面直行座標に変換する．
 * cf. https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
 *
 * @param latitude 北緯[deg]
 * @param longitude 東経[deg]
 * @param latitude_0 原点の北緯[deg]
 * @param longitude_0 原点の東経[deg]
 * @param x 北向きを正と定めたときのX座標[m] (出力)
 * @param y 西向きを正と定めたときのY座標[m] (出力)
 */
void gnssToCartRelative(
  const double& latitude,
  const double& longitude,
  const double& latitude_0,
  const double& longitude_0,
  double& x,
  double& y);

/**
 * @brief 平面直行座標を緯度，経度に変換する．
 * cf. https://qiita.com/sw1227/items/e7a590994ad7dcd0e8ab
 *
 * @param x 北向きを正と定めたときのX座標[m]
 * @param y 西向きを正と定めたときのY座標[m]
 * @param latitude_0 原点の北緯[deg]
 * @param longitude_0 原点の東経[deg]
 * @param latitude 北緯[deg] (出力)
 * @param longitude 東経[deg] (出力)
 */
void cartToGnssRelative(
  const double& x,
  const double& y,
  const double& latitude_0,
  const double& longitude_0,
  double& latitude,
  double& longitude);
}  // namespace tobas_std
