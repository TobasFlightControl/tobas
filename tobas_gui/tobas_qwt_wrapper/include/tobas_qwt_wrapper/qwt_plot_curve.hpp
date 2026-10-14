// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <qwt/qwt_plot.h>
#include <QColor>

class QwtPlotCurve;

namespace tobas
{
namespace qwt
{
class QwtPlotCurveWrapper
{
public:
  QwtPlotCurveWrapper(const QString& _title = "");
  QwtPlotCurveWrapper(const char* _title = "");

  void setPen(const QColor& _color, qreal _width, Qt::PenStyle _style = Qt::SolidLine);

  void setSamples(const QVector<double>& _x_data, const QVector<double>& _y_data);

  void setStyleLines();
  void setStyleSticks();
  void setStyleSteps();
  void setStyleDots();

  void attach(QwtPlot* _plot);

  /** Clear samples. */
  void clear();

private:
  std::shared_ptr<QwtPlotCurve> impl_;
};
}  // namespace qwt
}  // namespace tobas
