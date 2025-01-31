#pragma once

#include <qwt/qwt_plot.h>

namespace qt
{
/**
 * ===== QwtPlotとの違い =====
 * - 最小化可能
 * - SizePolicyをPreferredに
 * - 追加メソッド
 */
class QwtPlot2 : public QwtPlot
{
  Q_OBJECT

  using self = QwtPlot2;
  using super = QwtPlot;

public:
  explicit QwtPlot2(QWidget* parent = nullptr);

  /* プロット内部に凡例を追加． */
  void innerLegend();
};
}  // namespace qt
