// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include <tobas_dsp/welford.hpp>
#include <tobas_eigen_tools/core.hpp>

int main(int _argc, char* _argv[])
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <Data Length>\n";
    return EXIT_FAILURE;
  }

  const auto length = std::stoul(_argv[1]);

  // Define random generator.
  std::random_device rnd_dev;
  std::mt19937 rnd_gen(rnd_dev());
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  // Create data.
  std::vector<Eigen::Vector3d> data;
  for (std::size_t _ = 0; _ < length; ++_) {
    data.emplace_back(uniform(rnd_gen), uniform(rnd_gen), uniform(rnd_gen));
  }

  // Compute variance with normal method.
  Eigen::Vector3d data_sum = Eigen::Vector3d::Zero();
  for (const auto& x : data) {
    data_sum += x;
  }
  const Eigen::Vector3d mean_1 = data_sum / length;

  Eigen::Matrix3d cov_sum = Eigen::Matrix3d::Zero();
  for (const auto& x : data) {
    const Eigen::Vector3d d = x - mean_1;
    cov_sum += d * d.transpose();
  }
  const Eigen::Matrix3d cov_1 = cov_sum / length;

  // Compute variance with Welford method.
  tobas::dsp::Welford<double, 3> welford;
  for (const auto& x : data) {
    welford.add(x);
  }
  const auto& mean_2 = welford.mean();
  const auto& cov_2 = welford.variance();

  // Show results.
  std::cout << "Mean (Normal Method) : " << mean_1.transpose() << '\n';
  std::cout << "Mean (Welford Method): " << mean_2.transpose() << '\n';
  std::cout << "Coariance (Normal Method):\n" << cov_1 << '\n';
  std::cout << "Coariance (Welford Method):\n" << cov_2 << '\n';

  // Validate.
  if (!tobas::eigen::isClose(mean_1, mean_2) || !tobas::eigen::isClose(cov_1, cov_2)) {
    std::cerr << "Welford method is inaccurate.\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
