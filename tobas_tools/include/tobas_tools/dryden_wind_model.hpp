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

  inline const double& scaleLengthLon() const;
  inline const double& scaleLengthLat() const;
  inline const double& scaleLengthVer() const;
  inline const double& intensityLon() const;
  inline const double& intensityLat() const;
  inline const double& intensityVer() const;
  inline const double& updateRateLon() const;
  inline const double& updateRateLat() const;
  inline const double& updateRateVer() const;
  inline double noiseStddevLon() const;
  inline double noiseStddevLat() const;
  inline double noiseStddevVer() const;

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

  inline const double& u() const;
  inline const double& v() const;
  inline const double& w() const;

private:
  double u_ = 0., v_ = 0., w_ = 0.;  // 機体座標系で見た突風成分

  DrydenComponents components_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  std::normal_distribution<double> noise_;
};

inline const double& DrydenComponents::scaleLengthLon() const
{
  return L_uv_;
}

inline const double& DrydenComponents::scaleLengthLat() const
{
  return L_uv_;
}

inline const double& DrydenComponents::scaleLengthVer() const
{
  return L_w_;
}

inline const double& DrydenComponents::intensityLon() const
{
  return sigma_uv_;
}

inline const double& DrydenComponents::intensityLat() const
{
  return sigma_uv_;
}

inline const double& DrydenComponents::intensityVer() const
{
  return sigma_w_;
}

inline const double& DrydenComponents::updateRateLon() const
{
  return r_uv_;
}

inline const double& DrydenComponents::updateRateLat() const
{
  return r_uv_;
}

inline const double& DrydenComponents::updateRateVer() const
{
  return r_w_;
}

inline double DrydenComponents::noiseStddevLon() const
{
  return sqrt(2 * r_uv_) * sigma_uv_;
}

inline double DrydenComponents::noiseStddevLat() const
{
  return sqrt(2 * r_uv_) * sigma_uv_;
}

inline double DrydenComponents::noiseStddevVer() const
{
  return sqrt(2 * r_w_) * sigma_w_;
}

inline const double& DrydenSimulator::u() const
{
  return u_;
}

inline const double& DrydenSimulator::v() const
{
  return v_;
}

inline const double& DrydenSimulator::w() const
{
  return w_;
}
}  // namespace tobas
