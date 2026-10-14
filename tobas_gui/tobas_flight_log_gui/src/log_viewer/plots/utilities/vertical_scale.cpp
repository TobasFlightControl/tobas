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
/** Resolved axis limits and the interval between major ticks. */
struct VerticalScale
{
  double min;
  double max;
  double step;
};

/** Expand a data range to Qwt-aligned limits, using [-1, 1] when empty. */
VerticalScale makeVerticalScale(const VerticalScaleRange& _range)
{
  VerticalScale scale{ _range.empty() ? -1.0 : _range.min(), _range.empty() ? 1.0 : _range.max(), 0.0 };
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps, scale.min, scale.max, scale.step);
  return scale;
}

/** Apply a resolved scale to a plot's vertical axis. */
void applyVerticalScale(QwtPlot2& _plot, const VerticalScale& _scale)
{
  _plot.setAxisScale(QwtPlot::yLeft, _scale.min, _scale.max, _scale.step);
}
}  // namespace

void VerticalScaleRange::include(double _value)
{
  if (std::isfinite(_value)) {
    min_ = std::min(min_, _value);
    max_ = std::max(max_, _value);
  }
}

void VerticalScaleRange::include(const VerticalScaleRange& _other)
{
  if (!_other.empty()) {
    include(_other.min_);
    include(_other.max_);
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

void setVerticalScale(QwtPlot2& _plot, const VerticalScaleRange& _range, double _minimum_half_range)
{
  assert(_minimum_half_range >= 0.0);

  // Extend the input symmetrically before Qwt aligns the limits and major tick step.
  auto scale_range = _range;
  const auto center = _range.center();
  scale_range.include(center - _minimum_half_range);
  scale_range.include(center + _minimum_half_range);

  applyVerticalScale(_plot, makeVerticalScale(scale_range));
}

void setTargetCenteredVerticalScale(
  QwtPlot2& _plot,
  const VerticalScaleRange& _range,
  const VerticalScaleRange& _target_range,
  double _minimum_half_range)
{
  auto scale_range = _range;
  scale_range.include(_target_range);

  // Fall back to the current range when no target sample is available.
  const auto& center_range = _target_range.empty() ? _range : _target_range;
  const auto center = center_range.center();

  auto half_range = _minimum_half_range;
  if (!scale_range.empty()) {
    // Extend both sides to the farthest sample so that the input range remains symmetric.
    half_range = std::max({ half_range, std::abs(scale_range.min() - center), std::abs(scale_range.max() - center) });
  }

  // Qwt rounds the symmetric input range and selects a readable major tick step.
  double scale_min = center - half_range;
  double scale_max = center + half_range;
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps, scale_min, scale_max, scale_step);

  applyVerticalScale(_plot, { scale_min, scale_max, scale_step });
}

void setSharedVerticalScale(std::span<QwtPlot2* const> _plots, const VerticalScaleRange& _range)
{
  const auto scale = makeVerticalScale(_range);
  for (auto& plot : _plots) {
    applyVerticalScale(*plot, scale);
  }
}

void setSharedZeroCenteredVerticalScale(std::span<QwtPlot2* const> _plots, const VerticalScaleRange& _range)
{
  double scale_min = 0.0;
  double scale_max = _range.empty() ? 1.0 : std::max(std::abs(_range.min()), std::abs(_range.max()));
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps / 2, scale_min, scale_max, scale_step);

  // Reflect the larger auto-scaled extent so that zero remains at the exact center.
  scale_max = std::max(std::abs(scale_min), std::abs(scale_max));
  const VerticalScale scale{ -scale_max, scale_max, scale_step };
  for (auto& plot : _plots) {
    applyVerticalScale(*plot, scale);
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
