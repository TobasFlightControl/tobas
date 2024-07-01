#pragma once

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
}  // namespace tobas
