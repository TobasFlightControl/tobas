// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <array>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include <tobas_dsp/moving_stat.hpp>
#include <tobas_eigen_tools/core.hpp>

int main(int _argc, char* _argv[])
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <Steps>\n";
    return EXIT_FAILURE;
  }

  const auto steps = std::stoul(_argv[1]);

  // Define random generator.
  std::random_device rnd_dev;
  std::mt19937 rnd_gen(rnd_dev());
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  // Create data.
  constexpr std::size_t kWindowLength = 1000;
  std::array<Eigen::Vector3d, kWindowLength> init_data;
  std::vector<Eigen::Vector3d> incoming_data(steps);
  std::vector<Eigen::Vector3d> total_data(kWindowLength + steps);
  for (std::size_t i = 0; i < kWindowLength; ++i) {
    const Eigen::Vector3d data(uniform(rnd_gen), uniform(rnd_gen), uniform(rnd_gen));
    init_data.at(i) = data;
    total_data.at(i) = data;
  }
  for (std::size_t i = 0; i < steps; ++i) {
    const Eigen::Vector3d data(uniform(rnd_gen), uniform(rnd_gen), uniform(rnd_gen));
    incoming_data.at(i) = data;
    total_data.at(kWindowLength + i) = data;
  }

  // Initialize MovingStatistics object.
  tobas::dsp::MovingStatistics<double, 3, kWindowLength> moving_stat;
  moving_stat.initialize(init_data);

  for (std::size_t i = 0; i < steps; ++i) {
    // Compute moving statistics with offline method.
    Eigen::Vector3d data_sum = Eigen::Vector3d::Zero();
    for (std::size_t j = i; j < i + kWindowLength; ++j) {
      data_sum += total_data.at(j);
    }
    const Eigen::Vector3d mean_1 = data_sum / kWindowLength;

    Eigen::Matrix3d cov_sum = Eigen::Matrix3d::Zero();
    for (std::size_t j = i; j < i + kWindowLength; ++j) {
      const Eigen::Vector3d d = total_data.at(j) - mean_1;
      cov_sum += d * d.transpose();
    }
    const Eigen::Matrix3d cov_1 = cov_sum / kWindowLength;

    // Compute moving statistics with online method.
    const auto mean_2 = moving_stat.mean();
    const auto cov_2 = moving_stat.variance();

    // Show results.
    std::cout << "Start Index: " << i << ", Window Size: " << kWindowLength << '\n';
    std::cout << "Mean (Normal Method): " << mean_1.transpose() << '\n';
    std::cout << "Mean (Moving Stat)  : " << mean_2.transpose() << '\n';
    std::cout << "Covariance (Normal Method):\n" << cov_1 << '\n';
    std::cout << "Covariance (Moving Stat)  :\n" << cov_2 << '\n';
    std::cout << "----------\n";

    // Validate.
    if (!tobas::eigen::isClose(mean_1, mean_2) || !tobas::eigen::isClose(cov_1, cov_2)) {
      std::cerr << "Moving statistics is inaccurate.\n";
      return EXIT_FAILURE;
    }

    // Add next data.
    moving_stat.add(incoming_data[i]);
  }

  return EXIT_SUCCESS;
}
