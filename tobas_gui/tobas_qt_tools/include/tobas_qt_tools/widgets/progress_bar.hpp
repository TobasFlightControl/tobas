// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QColor>
#include <QProgressBar>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QProgressBar` =====
 * - Text is visible and centered by default.
 * - Additional methods for display-only telemetry bars.
 */
class ProgressBar : public QProgressBar
{
  using super = QProgressBar;

public:
  /* Constructs a progress bar with text shown at the center. */
  explicit ProgressBar(QWidget* parent = nullptr);

  /* Clears the displayed text and resets the value to the minimum. */
  void reset();

  /* Sets the bar fill color. */
  void setFillColor(const QColor& color);

  /* Sets the value as a percentage in the current progress range. */
  void setPercentage(double percentage);
};
}  // namespace qt
}  // namespace tobas
