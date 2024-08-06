#include <rclcpp/rclcpp.hpp>

#include <tobas_std_tools/time.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>
#include <tobas_geomag/model_params.hpp>

#include "../include/tobas_tools/utils.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
geomag::Elements geomag(const double& lat, const double& lon, const double& height)
{
  const auto year_frac = tobas_std::yearFraction();
  if (year_frac < 2020)
    throw runtime_error("Invalid year: " + to_string(year_frac));

  // 5年ごとに新しいデータが出るので，それを過ぎたら警告する
  // World Magnetic Model: https://www.ncei.noaa.gov/products/world-magnetic-model
  if (year_frac - 2020 > 5)
    PRINT_WARN("It is time to replace the WMM data with the latest version.");

  const auto ecef = geomag::ecefFromGeodetic(lat, lon, height);  // FIXME: geomagの高度は海抜ではなく楕円体
  const auto mag_field = geomag::magFieldFromECEF(year_frac, ecef, geomag::WMM2020);
  return geomag::elementsFromMagField(mag_field, lat, lon);
}
}  // namespace tobas
