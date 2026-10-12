// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <random>

class DepthNoiseModel
{
public:
  explicit DepthNoiseModel(float min_depth, float max_depth);

  virtual void applyNoise(size_t width, size_t height, float* data) = 0;

  const float bad_point_;

  std::normal_distribution<float> noise_;
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  bool inRange(float depth) const;

private:
  /** Values smaller/larger than these two are replaced by NaN. */
  float min_depth_;  ///< [m]
  float max_depth_;  ///< [m]
};

class KinectDepthNoiseModel : public DepthNoiseModel
{
  using super = DepthNoiseModel;

public:
  explicit KinectDepthNoiseModel(float min_depth, float max_depth);

  void applyNoise(size_t width, size_t height, float* data) override;
};

class PMDDepthNoiseModel : public DepthNoiseModel
{
  using super = DepthNoiseModel;

public:
  explicit PMDDepthNoiseModel(float min_depth, float max_depth);

  void applyNoise(size_t width, size_t height, float* data) override;
};

class D435DepthNoiseModel : public DepthNoiseModel
{
  using super = DepthNoiseModel;

public:
  explicit D435DepthNoiseModel(float min_depth, float max_depth, float horizontal_fov, float baseline);

  void applyNoise(size_t width, size_t height, float* data) override;

private:
  const float horizontal_fov_;
  const float baseline_;
};
