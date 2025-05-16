#include "tobas_wind_model/dryden.hpp"

#include <cassert>

#include <tobas_constants/constants.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;

namespace tobas
{
DrydenComponents::DrydenComponents()
{
}

void DrydenComponents::update(const double& relative_wind_speed, const double& altitude, const double& dt)
{
  assert(relative_wind_speed >= 0);
  assert(dt >= 0);

  const auto h = max(altitude, dryden::kMinimumAltitude);  // 高度が正であることを保証
  const auto h_ft = h * tobas_std::kMeterToFeet;
  if (h_ft > dryden::kLowAltitudeThreshold) {
    PRINT_WARN(
      "Since the altitude from the ground exceeds " << dryden::kLowAltitudeThreshold
                                                    << " feet, the Dryden wind model might be inaccurate.");
  }

  const auto tmp = 0.177 + 0.000823 * h_ft;  // [-]
  L_w_ = h;
  L_uv_ = h / pow(tmp, 1.2);
  sigma_w_ = 0.1 * mean_speed_;
  sigma_uv_ = sigma_w_ / pow(tmp, 0.4);
  r_w_ = relative_wind_speed / L_w_ * dt;
  r_uv_ = relative_wind_speed / L_uv_ * dt;
}

void DrydenComponents::setMeanWindSpeed(const double& mean_wind_speed)
{
  assert(mean_wind_speed >= 0);
  mean_speed_ = mean_wind_speed;
}

DrydenSimulator::DrydenSimulator() : rnd_gen_(rnd_dev_()), noise_(0, 1)
{
}

void DrydenSimulator::update(const double& relative_wind_speed, const double& altitude, const double& dt)
{
  components_.update(relative_wind_speed, altitude, dt);

  // 乱流を更新
  // 突風の波長が一定の場合，相対的な風速が大きいほど周波数が大きくなる (ドップラー効果)
  u_ = (1 - components_.updateRateLon()) * u_ + components_.noiseStddevLon() * noise_(rnd_gen_);
  v_ = (1 - components_.updateRateLat()) * v_ + components_.noiseStddevLat() * noise_(rnd_gen_);
  w_ = (1 - components_.updateRateVer()) * w_ + components_.noiseStddevVer() * noise_(rnd_gen_);
}

void DrydenSimulator::setMeanWindSpeed(const double& mean_wind_speed)
{
  components_.setMeanWindSpeed(mean_wind_speed);
}
}  // namespace tobas
