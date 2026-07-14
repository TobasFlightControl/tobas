// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/utilities/vertical_scale.hpp"

#include <algorithm>
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
void setVerticalScale(std::span<QwtPlot2* const> plots, double scale_min, double scale_max, double scale_step)
{
  for (auto& plot : plots) {
    plot->setAxisScale(QwtPlot::yLeft, scale_min, scale_max, scale_step);
  }
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

void setSharedVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range)
{
  double scale_min = range.empty() ? -1.0 : range.min();
  double scale_max = range.empty() ? 1.0 : range.max();
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps, scale_min, scale_max, scale_step);

  setVerticalScale(plots, scale_min, scale_max, scale_step);
}

void setSharedZeroCenteredVerticalScale(std::span<QwtPlot2* const> plots, const VerticalScaleRange& range)
{
  double scale_min = 0.0;
  double scale_max = range.empty() ? 1.0 : std::max(std::abs(range.min()), std::abs(range.max()));
  double scale_step = 0.0;
  QwtLinearScaleEngine().autoScale(kMaxVerticalScaleSteps / 2, scale_min, scale_max, scale_step);

  scale_max = std::max(std::abs(scale_min), std::abs(scale_max));
  setVerticalScale(plots, -scale_max, scale_max, scale_step);
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
