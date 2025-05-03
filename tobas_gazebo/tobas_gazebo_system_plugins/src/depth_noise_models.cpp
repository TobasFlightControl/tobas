#include <algorithm>
#include <iostream>
#include <eigen3/Eigen/Core>

#include "../include/tobas_gazebo_system_plugins/depth_noise_models.hpp"

using namespace std;
using namespace Eigen;

DepthNoiseModel::DepthNoiseModel(const float& min_depth, const float& max_depth)
  : bad_point_(numeric_limits<float>::quiet_NaN()), rnd_gen_(rnd_dev_()), min_depth_(min_depth), max_depth_(max_depth)
{
}

bool DepthNoiseModel::inRange(const float& depth) const
{
  return min_depth_ < depth && depth < max_depth_;
}

KinectDepthNoiseModel::KinectDepthNoiseModel(const float& min_depth, const float& max_depth)
  : super(min_depth, max_depth)
{
}

void KinectDepthNoiseModel::applyNoise(const size_t& width, const size_t& height, float* data)
{
  if (!data) {
    return;
  }

  // Axial noise model from
  // https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6375037,
  // Nguyen, Izadi & Lovell: "Modeling Kinect Sensor Noise for Improved 3D Reconstrucion and
  // Tracking", 3DIM/3DPVT, 2012. We are using the 10-60 Degree model as an approximation.
  Map<VectorXf> data_vector_map(data, width * height);
  VectorXf var_noise = 1.2e-3f + 1.9e-3f * (data_vector_map.array() - 0.4f).array().square();

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (size_t i = 0; i < width * height; ++i) {
    if (inRange(data_vector_map[i])) {
      data_vector_map[i] += noise_(rnd_gen_) * var_noise(i);
    }
    else {
      data_vector_map[i] = bad_point_;
    }
  }
}

PMDDepthNoiseModel::PMDDepthNoiseModel(const float& min_depth, const float& max_depth) : super(min_depth, max_depth)
{
}

void PMDDepthNoiseModel::applyNoise(const size_t& width, const size_t& height, float* data)
{
  if (!data) {
    return;
  }

  // 1% error claimed by PMD
  Map<VectorXf> data_vector_map(data, width * height);
  VectorXf var_noise = data_vector_map.array() * 0.01f;

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (size_t i = 0; i < width * height; ++i) {
    if (inRange(data_vector_map[i])) {
      data_vector_map[i] += noise_(rnd_gen_) * var_noise(i);
    }
    else {
      data_vector_map[i] = bad_point_;
    }
  }
}

D435DepthNoiseModel::D435DepthNoiseModel(float min_depth, float max_depth, float horizontal_fov, float baseline)
  : super(min_depth, max_depth), horizontal_fov_(horizontal_fov), baseline_(baseline)
{
}

void D435DepthNoiseModel::applyNoise(const size_t& width, const size_t& height, float* data)
{
  if (!data) {
    return;
  }

  float f = 0.5f * (width / tanf(horizontal_fov_ / 2.0f));
  float multiplier = (SubpixelErr) / (f * baseline_ * 1e+6f);
  Map<VectorXf> data_vector_map(data, width * height);

  // Formula taken from the Intel Whitepaper:
  // "Best-Known-Methods for Tuning Intel RealSense™ D400 Depth Cameras for Best Performance".
  // We are using the theoretical RMS model formula.
  VectorXf rms_noise = (data_vector_map * 1e+3f).array().square() * multiplier;
  VectorXf noise = rms_noise.array().square();

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (size_t i = 0; i < width * height; ++i) {
    if (inRange(data_vector_map[i])) {
      data_vector_map[i] += noise_(rnd_gen_) * min(((float)noise(i)), MaxStddev);
    }
    else {
      data_vector_map[i] = bad_point_;
    }
  }
}
