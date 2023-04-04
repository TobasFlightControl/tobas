#pragma once

/**
 * @brief 3次元ベクトルの回転
 *
 * @param roll,pitch,yaw R_a_b (in)
 * @param xb,yb,zb V_b (in)
 * @param xa,ya,za V_a (out)
 */
void rotateVector(
  double roll,
  double pitch,
  double yaw,
  double xb,
  double yb,
  double zb,
  double& xa,
  double& ya,
  double& za);
