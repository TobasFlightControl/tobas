#pragma once

#include <qwt/qwt_plot.h>

namespace gui
{
namespace log
{
class QwtPlot2 : public QwtPlot
{
  Q_OBJECT

  using self = QwtPlot2;
  using super = QwtPlot;

public:
  explicit QwtPlot2(QWidget* parent = nullptr);

  void setAxisNoLabel(const QwtPlot::Axis& axis);
  void setAxisLabelUnit(const QwtPlot::Axis& axis, const QString& unit);

  void setupBinaryPlot(const QString& label0, const QString& label1);
};
}  // namespace log
}  // namespace gui
