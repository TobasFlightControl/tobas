#pragma once

#include <memory>

#include <QColor>
#include <qwt/qwt_plot.h>

class QwtPlotCurve;

namespace qwt
{
class QwtPlotCurveWrapper
{
public:
  QwtPlotCurveWrapper(const QString& title = "");
  QwtPlotCurveWrapper(const char* title = "");

  void setPen(const QColor& color, qreal width, Qt::PenStyle style = Qt::SolidLine);

  void setSamples(const QVector<double>& x_data, const QVector<double>& y_data);

  void attach(QwtPlot* plot);

private:
  std::shared_ptr<QwtPlotCurve> impl_;
};
}  // namespace qwt
