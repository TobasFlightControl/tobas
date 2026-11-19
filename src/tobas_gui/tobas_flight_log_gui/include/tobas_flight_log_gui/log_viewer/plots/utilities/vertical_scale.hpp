// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>

namespace tobas::gui::log
{
class QwtPlot2;

class VerticalScaleRange
{
public:
  /** Extend the range with a value, ignoring NaN and infinity. */
  inline void include(double _value)
  {
    if (std::isfinite(_value)) {
      min_ = std::min(min_, _value);
      max_ = std::max(max_, _value);
    }
  }

  /** Merge another nonempty range into this range. */
  inline void include(const VerticalScaleRange& _other)
  {
    if (!_other.empty()) {
      include(_other.min_);
      include(_other.max_);
    }
  }

  /** Check whether the range contains no finite values. */
  inline bool empty() const
  {
    return max_ < min_;
  }

  /** Get the minimum value from a nonempty range. */
  inline double min() const
  {
    return min_;
  }

  /** Get the maximum value from a nonempty range. */
  inline double max() const
  {
    return max_;
  }

  /** Get the range midpoint, or zero when the range is empty. */
  inline double center() const
  {
    return empty() ? 0.0 : (max() + min()) / 2.0;
  }

private:
  double min_ = std::numeric_limits<double>::max();
  double max_ = std::numeric_limits<double>::lowest();
};

/** Configure a plot's vertical scale from a value range and minimum half-range. */
void setVerticalScale(QwtPlot2& _plot, const VerticalScaleRange& _range, double _minimum_half_range = 0.0);

/** Configure a target-centered scale containing current and target values with a minimum half-range. */
void setTargetCenteredVerticalScale(
  QwtPlot2& _plot,
  const VerticalScaleRange& _range,
  const VerticalScaleRange& _target_range,
  double _minimum_half_range);

/** Apply one automatically resolved vertical scale to multiple plots. */
void setSharedVerticalScale(std::span<QwtPlot2* const> _plots, const VerticalScaleRange& _range);

/** Apply one shared zero-centered vertical scale to multiple plots. */
void setSharedZeroCenteredVerticalScale(std::span<QwtPlot2* const> _plots, const VerticalScaleRange& _range);
}  // namespace tobas::gui::log
