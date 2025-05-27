#include "tobas_qwt_wrapper/qwt_plot_curve.hpp"

#include <qwt/qwt_plot_curve.h>  // XXX: <= C++17

namespace qwt
{
QwtPlotCurveWrapper::QwtPlotCurveWrapper(const QString& title)
{
  impl_ = std::make_shared<QwtPlotCurve>(title);
}

QwtPlotCurveWrapper::QwtPlotCurveWrapper(const char* title) : QwtPlotCurveWrapper(QString(title))
{
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
