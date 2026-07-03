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
  QwtPlotCurveWrapper(const QString& title = "");
  QwtPlotCurveWrapper(const char* title = "");

  void setPen(const QColor& color, qreal width, Qt::PenStyle style = Qt::SolidLine);

  void setSamples(const QVector<double>& x_data, const QVector<double>& y_data);

  void setStyleLines();
  void setStyleSticks();
  void setStyleSteps();
  void setStyleDots();

  void attach(QwtPlot* plot);

  /* Clear samples. */
  void clear();

private:
  std::shared_ptr<QwtPlotCurve> impl_;
};
}  // namespace qwt
}  // namespace tobas
