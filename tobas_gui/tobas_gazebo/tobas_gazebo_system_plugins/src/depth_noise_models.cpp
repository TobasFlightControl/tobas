// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/depth_noise_models.hpp"

#include <algorithm>

#include <eigen3/Eigen/Core>

DepthNoiseModel::DepthNoiseModel(float _min_depth, float _max_depth)
  : bad_point_(std::numeric_limits<float>::quiet_NaN())
  , rnd_gen_(rnd_dev_())
  , min_depth_(_min_depth)
  , max_depth_(_max_depth)
{
}

bool DepthNoiseModel::inRange(float _depth) const
{
  return min_depth_ < _depth && _depth < max_depth_;
}

KinectDepthNoiseModel::KinectDepthNoiseModel(float _min_depth, float _max_depth) : super(_min_depth, _max_depth)
{
}

void KinectDepthNoiseModel::applyNoise(size_t _width, size_t _height, float* _data)
{
  if (!_data) {
    return;
  }

  // Axial noise model from
  // Ref: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6375037,
  // Nguyen, Izadi & Lovell: "Modeling Kinect Sensor Noise for Improved 3D Reconstrucion and
  // Tracking", 3DIM/3DPVT, 2012. We are using the 10-60 Degree model as an approximation.
  Eigen::Map<Eigen::VectorXf> data_vector_map(_data, _width * _height);
  const Eigen::VectorXf var_noise = 1.2e-3f + 1.9e-3f * (data_vector_map.array() - 0.4f).array().square();

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (size_t i = 0; i < _width * _height; ++i) {
    if (inRange(data_vector_map[i])) {
      data_vector_map[i] += noise_(rnd_gen_) * var_noise(i);
    }
    else {
      data_vector_map[i] = bad_point_;
    }
  }
}

PMDDepthNoiseModel::PMDDepthNoiseModel(float _min_depth, float _max_depth) : super(_min_depth, _max_depth)
{
}

void PMDDepthNoiseModel::applyNoise(size_t _width, size_t _height, float* _data)
{
  if (!_data) {
    return;
  }

  // 1% error claimed by PMD
  Eigen::Map<Eigen::VectorXf> data_vector_map(_data, _width * _height);
  const Eigen::VectorXf var_noise = data_vector_map.array() * 0.01f;

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (size_t i = 0; i < _width * _height; ++i) {
    if (inRange(data_vector_map[i])) {
      data_vector_map[i] += noise_(rnd_gen_) * var_noise(i);
    }
    else {
      data_vector_map[i] = bad_point_;
    }
  }
}

D435DepthNoiseModel::D435DepthNoiseModel(float _min_depth, float _max_depth, float _horizontal_fov, float _baseline)
  : super(_min_depth, _max_depth), horizontal_fov_(_horizontal_fov), baseline_(_baseline)
{
}

void D435DepthNoiseModel::applyNoise(size_t _width, size_t _height, float* _data)
{
  if (!_data) {
    return;
  }

  constexpr float kSubpixelErr = 0.1f;  // [px] Calibration error
  const auto f = 0.5f * (_width / std::tan(horizontal_fov_ / 2.0f));
  const auto multiplier = kSubpixelErr / (f * baseline_ * 1e+6f);
  Eigen::Map<Eigen::VectorXf> data_vector_map(_data, _width * _height);

  // Formula taken from the Intel Whitepaper:
  // "Best-Known-Methods for Tuning Intel RealSense™ D400 Depth Cameras for Best Performance".
  // We are using the theoretical RMS model formula.
  const Eigen::VectorXf rms_noise = (data_vector_map * 1e+3f).array().square() * multiplier;
  const Eigen::VectorXf noise = rms_noise.array().square();

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (size_t i = 0; i < _width * _height; ++i) {
    if (inRange(data_vector_map[i])) {
      constexpr float kMaxStddev = 3.0f;  // [m] cutoff for distance standard deviation
      data_vector_map[i] += noise_(rnd_gen_) * std::min(noise(i), kMaxStddev);
    }
    else {
      data_vector_map[i] = bad_point_;
    }
  }
}
