// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <eigen3/Eigen/Core>

#include <tobas_time_tools/stopwatch.hpp>

template <typename T>
void process(int _n, int _iter)
{
  const auto A = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Random(_n, _n);
  const auto B = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Random(_n, _n);

  tobas::tim::Stopwatch stopwatch(_iter);

  for (int i = 0; i < _iter; ++i) {
    std::cout << "Iter " << i << '\n';

    stopwatch.start();
    const auto C = (A * B).eval();
    stopwatch.stop();
  }
}

int main(int _argc, char* _argv[])
{
  if (_argc < 4) {
    std::cerr << "Usage: " << _argv[0] << " <Scalar> <Matrix Size> <Iterations>\n";
    return EXIT_FAILURE;
  }

  const auto scalar = _argv[1];
  const auto n = atoi(_argv[2]);
  const auto iter = atoi(_argv[3]);

  if (std::strcmp(scalar, "char") == 0) {
    process<char>(n, iter);
  }
  else if (std::strcmp(scalar, "short") == 0) {
    process<short>(n, iter);
  }
  else if (std::strcmp(scalar, "int") == 0) {
    process<int>(n, iter);
  }
  else if (std::strcmp(scalar, "long") == 0) {
    process<long>(n, iter);
  }
  else if (std::strcmp(scalar, "float") == 0) {
    process<float>(n, iter);
  }
  else if (std::strcmp(scalar, "double") == 0) {
    process<double>(n, iter);
  }
  else {
    std::cerr << "Invalid scalar type: " << scalar << '\n';
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
