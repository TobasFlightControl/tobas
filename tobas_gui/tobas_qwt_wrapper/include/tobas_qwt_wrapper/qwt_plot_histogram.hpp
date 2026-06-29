// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <qwt/qwt_plot.h>
#include <QColor>

class QwtPlotHistogram;

namespace tobas
{
namespace qwt
{
class QwtPlotHistogramWrapper
{
public:
  explicit QwtPlotHistogramWrapper(const QString& title = "");

  void setPen(const QColor& color, qreal width, Qt::PenStyle style = Qt::SolidLine);
  void setBrush(const QColor& color);

  void setSamples(const QVector<double>& values, double min_value, double max_value, double bin_width);

  void attach(QwtPlot* plot);

  /* サンプルを消去する． */
  void clear();

private:
  std::shared_ptr<QwtPlotHistogram> impl_;
};
}  // namespace qwt
}  // namespace tobas
