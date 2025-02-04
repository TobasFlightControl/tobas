#include <qwt/qwt_plot_curve.h>  // <= C++17

#include "../include/tobas_qwt_wrapper/qwt_plot_curve.hpp"

namespace qwt
{
QwtPlotCurveWrapper::QwtPlotCurveWrapper(const QString& title)
{
  impl_ = new QwtPlotCurve(title);
}

QwtPlotCurveWrapper::~QwtPlotCurveWrapper()
{
  delete impl_;
}

void QwtPlotCurveWrapper::setPen(const QColor& color, qreal width, Qt::PenStyle style)
{
  impl_->setPen(color, width, style);
}

void QwtPlotCurveWrapper::setSamples(const QVector<double>& x_data, const QVector<double>& y_data)
{
  impl_->setSamples(x_data, y_data);
}

void QwtPlotCurveWrapper::attach(QwtPlot* plot)
{
  impl_->attach(plot);
}
}  // namespace qwt
