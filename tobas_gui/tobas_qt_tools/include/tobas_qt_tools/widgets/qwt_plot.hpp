#pragma once

#include <qwt/qwt_plot.h>

namespace qt
{
/**
 * ===== QwtPlotとの違い =====
 * - 最小化可能
 * - SizePolicyをPreferredに
 */
class QwtPlot2 : public QwtPlot
{
  Q_OBJECT

  using self = QwtPlot2;
  using super = QwtPlot;

public:
  explicit QwtPlot2(QWidget* parent = nullptr);
};
}  // namespace qt
