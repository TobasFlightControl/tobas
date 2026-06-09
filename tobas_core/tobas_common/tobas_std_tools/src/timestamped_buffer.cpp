// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_std_tools/timestamped_buffer.hpp"

#include <cmath>
#include <limits>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_math/core.hpp>

namespace tobas
{
namespace st
{
double TimestampedBufferDouble::max() const
{
  double res = -INFINITY;
  for (const auto& [_, x] : map_) {
    res = std::max(res, x);
  }
  return res;
}

double TimestampedBufferDouble::min() const
{
  double res = INFINITY;
  for (const auto& [_, x] : map_) {
    res = std::min(res, x);
  }
  return res;
}

double TimestampedBufferDouble::range() const
{
  return this->max() - this->min();
}

double TimestampedBufferDouble::mean() const
{
  if (this->size() == 0) {
    return 0.;
  }

  algo::Kahan<double> sum;
  for (const auto& [_, x] : map_) {
    sum.add(x);
  }

  return sum.get() / this->size();
}

double TimestampedBufferDouble::variance() const
{
  if (this->size() == 0) {
    return 0.;
  }

  const auto mean = this->mean();

  algo::Kahan<double> sum;
  for (const auto& [_, x] : map_) {
    sum.add(math::sqr(x - mean));
  }

  return sum.get() / this->size();
}

double TimestampedBufferDouble::stddev() const
{
  return std::sqrt(this->variance());
}
}  // namespace st
}  // namespace tobas
