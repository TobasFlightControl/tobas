#include <qwt/qwt_plot_legenditem.h>

#include "tobas_qt_tools/widgets/qwt_plot.hpp"

namespace qt
{
QwtPlot2::QwtPlot2(QWidget* parent) : super(parent)
{
  setMinimumSize(1, 1);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}

void QwtPlot2::innerLegend()
{
  const auto legend_item = new QwtPlotLegendItem();
  legend_item->attach(this);
  legend_item->setAlignment(Qt::AlignTop | Qt::AlignRight);  // 右上に配置
  legend_item->setMaxColumns(1);                             // 1列に並べる
  legend_item->setBackgroundMode(QwtPlotLegendItem::LegendBackground);
  legend_item->setBorderPen(QPen(Qt::black, 1));  // 黒の枠線
}
}  // namespace qt
