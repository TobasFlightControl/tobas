// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <qwt/qwt_plot.h>

namespace tobas::gui::log
{
class QwtPlot2 : public QwtPlot
{
  Q_OBJECT

  using super = QwtPlot;

public:
  explicit QwtPlot2(QWidget* _parent = nullptr);

  void setAxisNoLabel(QwtPlot::Axis _axis);
  void setAxisLabelUnit(QwtPlot::Axis _axis, const QString& _unit);

  void setupIndexedLabelPlot(const QStringList& _labels);
};
}  // namespace tobas::gui::log
