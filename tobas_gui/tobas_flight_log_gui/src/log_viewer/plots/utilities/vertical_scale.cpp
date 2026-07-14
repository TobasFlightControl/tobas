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

  for (auto& plot : plots) {
    plot->setAxisScale(QwtPlot::yLeft, scale_min, scale_max, scale_step);
  }
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
