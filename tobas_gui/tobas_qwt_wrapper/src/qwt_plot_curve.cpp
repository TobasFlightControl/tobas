// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qwt_wrapper/qwt_plot_curve.hpp"

#include <qwt/qwt_plot_curve.h>  // <= C++17

namespace tobas
{
namespace qwt
{
QwtPlotCurveWrapper::QwtPlotCurveWrapper(const QString& _title)
{
  impl_ = std::make_shared<QwtPlotCurve>(_title);
}

QwtPlotCurveWrapper::QwtPlotCurveWrapper(const char* _title) : QwtPlotCurveWrapper(QString(_title))
{
}

void QwtPlotCurveWrapper::setPen(const QColor& _color, qreal _width, Qt::PenStyle _style)
{
  impl_->setPen(_color, _width, _style);
}

void QwtPlotCurveWrapper::setSamples(const QVector<double>& _x_data, const QVector<double>& _y_data)
{
  impl_->setSamples(_x_data, _y_data);
}

void QwtPlotCurveWrapper::setStyleLines()
{
  impl_->setStyle(QwtPlotCurve::Lines);
}

void QwtPlotCurveWrapper::setStyleSticks()
{
  impl_->setStyle(QwtPlotCurve::Sticks);
}

void QwtPlotCurveWrapper::setStyleSteps()
{
  impl_->setStyle(QwtPlotCurve::Steps);
}

void QwtPlotCurveWrapper::setStyleDots()
{
  impl_->setStyle(QwtPlotCurve::Dots);
}

void QwtPlotCurveWrapper::attach(QwtPlot* _plot)
{
  impl_->attach(_plot);
}

void QwtPlotCurveWrapper::clear()
{
  QVector<double> empty;
  setSamples(empty, empty);
}
}  // namespace qwt
}  // namespace tobas
