// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/utilities/vertical_scale.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

#include <qwt/qwt_scale_engine.h>

#include "tobas_flight_log_gui/log_viewer/plots/utilities/constants.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/utilities/qwt_plot.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
namespace
{
/* Resolved axis limits and the interval between major ticks. */
struct VerticalScale
{
  double min;
  double max;
  double step;
};

/* Expand a data range to Qwt-aligned limits, using [-1, 1] when empty. */
VerticalScale makeVerticalScale(const VerticalScaleRange& range)
{
  VerticalScale scale{ range.empty() ? -1.0 : range.min(), range.empty() ? 1.0 : range.max(), 0.0 };
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps, scale.min, scale.max, scale.step);
  return scale;
}

/* Apply a resolved scale to a plot's vertical axis. */
void applyVerticalScale(QwtPlot2& plot, const VerticalScale& scale)
{
  plot.setAxisScale(QwtPlot::yLeft, scale.min, scale.max, scale.step);
}
}  // namespace

void VerticalScaleRange::include(double value)
{
  if (std::isfinite(value)) {
    min_ = std::min(min_, value);
    max_ = std::max(max_, value);
  }
}

void VerticalScaleRange::include(const VerticalScaleRange& other)
{
  if (!other.empty()) {
    include(other.min_);
    include(other.max_);
  }
}

bool VerticalScaleRange::empty() const
{
  return max_ < min_;
}

double VerticalScaleRange::min() const
{
  return min_;
}

double VerticalScaleRange::max() const
{
  return max_;
}

double VerticalScaleRange::center() const
{
  return empty() ? 0.0 : (max() + min()) / 2.0;
}

void setVerticalScale(QwtPlot2& plot, const VerticalScaleRange& range, double minimum_half_range)
{
  assert(minimum_half_range >= 0.0);

  // Extend the input symmetrically before Qwt aligns the limits and major tick step.
  auto scale_range = range;
  const auto center = range.center();
  scale_range.include(center - minimum_half_range);
  scale_range.include(center + minimum_half_range);

  applyVerticalScale(plot, makeVerticalScale(scale_range));
}

void setTargetCenteredVerticalScale(
  QwtPlot2& plot,
  const VerticalScaleRange& range,
  const VerticalScaleRange& target_range,
  double minimum_half_range)
{
  auto scale_range = range;
  scale_range.include(target_range);

  // Fall back to the current range when no target sample is available.
  const auto& center_range = target_range.empty() ? range : target_range;
  const auto center = center_range.center();

  auto half_range = minimum_half_range;
  if (!scale_range.empty()) {
    // Extend both sides to the farthest sample so that the input range remains symmetric.
    half_range = std::max({ half_range, std::abs(scale_range.min() - center), std::abs(scale_range.max() - center) });
  }

  // Qwt rounds the symmetric input range and selects a readable major tick step.
  double scale_min = center - half_range;
  double scale_max = center + half_range;
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps, scale_min, scale_max, scale_step);

  applyVerticalScale(plot, { scale_min, scale_max, scale_step });
}

void setSharedVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range)
{
  const auto scale = makeVerticalScale(range);
  for (auto& plot : plots) {
    applyVerticalScale(*plot, scale);
  }
}

void setSharedZeroCenteredVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range)
{
  double scale_min = 0.0;
  double scale_max = range.empty() ? 1.0 : std::max(std::abs(range.min()), std::abs(range.max()));
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps / 2, scale_min, scale_max, scale_step);

  // Reflect the larger auto-scaled extent so that zero remains at the exact center.
  scale_max = std::max(std::abs(scale_min), std::abs(scale_max));
  const VerticalScale scale{ -scale_max, scale_max, scale_step };
  for (auto& plot : plots) {
    applyVerticalScale(*plot, scale);
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
