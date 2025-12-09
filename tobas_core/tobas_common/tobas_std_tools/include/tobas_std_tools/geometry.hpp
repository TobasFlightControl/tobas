#pragma once

namespace tbs
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
}  // namespace tbs
