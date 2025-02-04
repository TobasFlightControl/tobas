#pragma once

#include <QColor>
#include <qwt/qwt_plot.h>

class QwtPlotCurve;

namespace qwt
{
class QwtPlotCurveWrapper
{
public:
  explicit QwtPlotCurveWrapper(const QString& title = "");
  ~QwtPlotCurveWrapper();

  void setPen(const QColor& color, qreal width, Qt::PenStyle style = Qt::SolidLine);

  void setSamples(const QVector<double>& x_data, const QVector<double>& y_data);

  void attach(QwtPlot* plot);

private:
  QwtPlotCurve* impl_;
};
}  // namespace qwt
