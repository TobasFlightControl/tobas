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
  explicit QwtPlotHistogramWrapper(const QString& _title = "");

  void setPen(const QColor& _color, qreal _width, Qt::PenStyle _style = Qt::SolidLine);
  void setBrush(const QColor& _color);

  void setSamples(const QVector<double>& _values, double _min_value, double _max_value, double _bin_width);

  void attach(QwtPlot* _plot);

  /** Clear samples. */
  void clear();

private:
  std::shared_ptr<QwtPlotHistogram> impl_;
};
}  // namespace qwt
}  // namespace tobas
