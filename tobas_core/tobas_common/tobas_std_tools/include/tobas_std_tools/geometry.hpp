#pragma once

#include <tuple>

namespace tbs
{
/**
 * @brief ZYX オイラー角 (Roll,Pitch,Yaw) からクォータニオン (X,Y,Z,W) を計算．
 * cf. https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
std::tuple<double, double, double, double>
quaternionFromEuler(const double& roll, const double& pitch, const double& yaw);

/**
 * @brief クォータニオン (X,Y,Z,W) から ZYX オイラー角 (Roll,Pitch,Yaw) を計算．
 * cf. https://qiita.com/aa_debdeb/items/abe90a9bd0b4809813da
 */
std::tuple<double, double, double>
eulerFromQuaternion(const double& x, const double& y, const double& z, const double& w);
}  // namespace tbs
