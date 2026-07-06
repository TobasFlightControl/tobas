// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/progress_bar.hpp"

#include <algorithm>
#include <cmath>

#include <QPalette>

namespace tobas
{
namespace qt
{
ProgressBar::ProgressBar(QWidget* parent) : super(parent)
{
  setTextVisible(true);
  setAlignment(Qt::AlignCenter);
  reset();
}

void ProgressBar::reset()
{
  setValue(minimum());
  setFormat("");
}

void ProgressBar::setFillColor(const QColor& color)
{
  auto pal = palette();
  pal.setColor(QPalette::Highlight, color);
  setPalette(pal);
}

void ProgressBar::setPercentage(double percentage)
{
  const auto clamped = std::clamp(percentage, static_cast<double>(minimum()), static_cast<double>(maximum()));
  setValue(static_cast<int>(std::lround(clamped)));
}
}  // namespace qt
}  // namespace tobas
