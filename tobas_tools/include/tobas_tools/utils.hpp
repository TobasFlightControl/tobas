#pragma once

#include <string>

#include <XYZgeomag.hpp>

#include "./drone.hpp"

namespace tobas
{
/* rosparamに登録されているdescriptionから質量 [kg] を求める． */
double getMass();

/**
 * @brief 経緯高度から地磁気の参照値を求める．
 *
 * @param lat 北緯 [deg]
 * @param lon 東経 [deg]
 * @param height WGS84楕円体上の高度 [m]
 * @return geomag::Elements
 */
geomag::Elements geomag(const double& lat, const double& lon, const double& height);

/* モータの目標回転数を有効範囲内にクランプし，範囲外だった場合は警告する．クランプが発生した場合にTrueを返す．*/
double clampTargetRotSpeedAndWarn(
  const Drone& drone,
  const size_t& rotor_idx,
  const double& battery_voltage,
  const double& tar_speed);
}  // namespace tobas
