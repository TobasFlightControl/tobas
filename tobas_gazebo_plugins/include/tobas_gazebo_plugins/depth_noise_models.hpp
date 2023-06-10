#pragma once

#include <random>

class DepthNoiseModel
{
public:
  explicit DepthNoiseModel(float min_depth, float max_depth);

  virtual void applyNoise(uint32_t width, uint32_t height, float* data) = 0;

protected:
  const float bad_point_;

  std::normal_distribution<float> noise_;
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  bool inRange(float depth) const;

private:
  // Values smaller/larger than these two are replaced by NaN
  float min_depth_;  // [m]
  float max_depth_;  // [m]
};

class KinectDepthNoiseModel : public DepthNoiseModel
{
  using super = DepthNoiseModel;

public:
  explicit KinectDepthNoiseModel(float min_depth, float max_depth);

  void applyNoise(uint32_t width, uint32_t height, float* data) override;
};

class PMDDepthNoiseModel : public DepthNoiseModel
{
  using super = DepthNoiseModel;

public:
  explicit PMDDepthNoiseModel(float min_depth, float max_depth);

  void applyNoise(uint32_t width, uint32_t height, float* data) override;
};

class D435DepthNoiseModel : public DepthNoiseModel
{
  static constexpr float SubpixelErr = 0.1f;  // [px] Calibration error
  static constexpr float MaxStddev = 3.0f;    // [m] cutoff for distance standard deviation

  using super = DepthNoiseModel;

public:
  explicit D435DepthNoiseModel(
    float min_depth,
    float max_depth,
    float horizontal_fov,
    float baseline);

  void applyNoise(uint32_t width, uint32_t height, float* data) override;

private:
  const float horizontal_fov_;
  const float baseline_;
};
