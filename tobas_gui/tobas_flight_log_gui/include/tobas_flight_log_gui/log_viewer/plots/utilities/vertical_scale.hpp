// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <limits>
#include <span>

namespace tobas
{
namespace gui
{
namespace log
{
class QwtPlot2;

class VerticalScaleRange
{
public:
  /* Extend the range with a value, ignoring NaN and infinity. */
  void include(double value);

  /* Merge another nonempty range into this range. */
  void include(const VerticalScaleRange& other);

  /* Check whether the range contains no finite values. */
  bool empty() const;

  /* Get the minimum value from a nonempty range. */
  double min() const;

  /* Get the maximum value from a nonempty range. */
  double max() const;

  /* Get the range midpoint, or zero when the range is empty. */
  double center() const;

private:
  double min_ = std::numeric_limits<double>::max();
  double max_ = std::numeric_limits<double>::lowest();
};

/* Configure a plot's vertical scale from a value range and minimum half-range. */
void setVerticalScale(QwtPlot2& plot, const VerticalScaleRange& range, double minimum_half_range = 0.0);

/* Configure a target-centered scale containing current and target values with a minimum half-range. */
void setTargetCenteredVerticalScale(
  QwtPlot2& plot,
  const VerticalScaleRange& range,
  const VerticalScaleRange& target_range,
  double minimum_half_range);

/* Apply one automatically resolved vertical scale to multiple plots. */
void setSharedVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range);

/* Apply one shared zero-centered vertical scale to multiple plots. */
void setSharedZeroCenteredVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range);
}  // namespace log
}  // namespace gui
}  // namespace tobas
