#include <cassert>

#include <dh_std_tools/console.hpp>

#include "../include/tobas_tools/dryden_wind_model.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
DrydenComponents::DrydenComponents(const double& mean_wind_speed)
{
  setMeanWindSpeed(mean_wind_speed);
}

void DrydenComponents::update(
  const double& relative_wind_speed,
  const double& altitude,
  const double& dt)
{
  assert(relative_wind_speed >= 0);
  assert(dt > 0);

  const auto h = max(altitude, dryden::kMinimumAltitude);  // 高度が正であることを保証
  const auto h_ft = h * kMeterToFeet;
  if (h_ft > dryden::kLowAltitudeThreshold)
  {
    DH_WARN(
      "Since the altitude from the ground exceeds "
      << dryden::kLowAltitudeThreshold << " feet, the Dryden wind model might be inaccurate.");
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

const double& DrydenComponents::scaleLengthLon() const
{
  return L_uv_;
}

const double& DrydenComponents::scaleLengthLat() const
{
  return L_uv_;
}

const double& DrydenComponents::scaleLengthVer() const
{
  return L_w_;
}

const double& DrydenComponents::intensityLon() const
{
  return sigma_uv_;
}

const double& DrydenComponents::intensityLat() const
{
  return sigma_uv_;
}

const double& DrydenComponents::intensityVer() const
{
  return sigma_w_;
}

const double& DrydenComponents::updateRateLon() const
{
  return r_uv_;
}

const double& DrydenComponents::updateRateLat() const
{
  return r_uv_;
}

const double& DrydenComponents::updateRateVer() const
{
  return r_w_;
}

double DrydenComponents::noiseStddevLon() const
{
  return sqrt(2 * r_uv_) * sigma_uv_;
}

double DrydenComponents::noiseStddevLat() const
{
  return sqrt(2 * r_uv_) * sigma_uv_;
}

double DrydenComponents::noiseStddevVer() const
{
  return sqrt(2 * r_w_) * sigma_w_;
}

DrydenSimulator::DrydenSimulator(const double& mean_wind_speed)
  : components_(mean_wind_speed), rnd_gen_(rnd_dev_()), noise_(0, 1)
{
}

void DrydenSimulator::update(
  const double& relative_wind_speed,
  const double& altitude,
  const double& dt)
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

const double& DrydenSimulator::u() const
{
  return u_;
}

const double& DrydenSimulator::v() const
{
  return v_;
}

const double& DrydenSimulator::w() const
{
  return w_;
}
}  // namespace tobas
