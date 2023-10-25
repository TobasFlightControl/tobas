#pragma once

#include <random>

namespace tobas
{
namespace dryden
{
static constexpr double kLowAltitudeThreshold = 1000.;  // [ft]
static constexpr double kMinimumAltitude = 1.;          // [m]
static constexpr double kDefaultMeanWindSpeed = 5.;     // [m/s]
};                                                      // namespace dryden

/**
 * @brief Dryden turbulance model
 * https://jp.mathworks.com/help/aeroblks/drydenwindturbulencemodeldiscrete.html
 */
class DrydenComponents
{
public:
  explicit DrydenComponents(const double& mean_wind_speed = dryden::kDefaultMeanWindSpeed);

  void update(const double& relative_wind_speed, const double& altitude, const double& dt);
  void setMeanWindSpeed(const double& mean_wind_speed);

  const double& scaleLengthLon() const;
  const double& scaleLengthLat() const;
  const double& scaleLengthVer() const;
  const double& intensityLon() const;
  const double& intensityLat() const;
  const double& intensityVer() const;
  const double& updateRateLon() const;
  const double& updateRateLat() const;
  const double& updateRateVer() const;
  double noiseStddevLon() const;
  double noiseStddevLat() const;
  double noiseStddevVer() const;

private:
  double mean_speed_;

  double L_uv_, L_w_;          // [m] 乱流のスケール長
  double sigma_uv_, sigma_w_;  // [m/s] 風速の標準偏差
  double r_uv_, r_w_;          // [-] 更新率
};

/**
 * @brief Dryden turbulance model
 * https://jp.mathworks.com/help/aeroblks/drydenwindturbulencemodeldiscrete.html
 */
class DrydenSimulator
{
public:
  explicit DrydenSimulator(const double& mean_wind_speed = dryden::kDefaultMeanWindSpeed);

  void update(const double& relative_wind_speed, const double& altitude, const double& dt);
  void setMeanWindSpeed(const double& mean_wind_speed);

  const double& u() const;
  const double& v() const;
  const double& w() const;

private:
  double u_ = 0., v_ = 0., w_ = 0.;  // 機体座標系で見た突風成分

  DrydenComponents components_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  std::normal_distribution<double> noise_;
};
}  // namespace tobas
