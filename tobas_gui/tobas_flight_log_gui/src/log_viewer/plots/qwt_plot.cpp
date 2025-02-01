#include <qwt/qwt_plot_legenditem.h>
#include <qwt/qwt_scale_draw.h>

#include "tobas_flight_log_gui/log_viewer/plots/qwt_plot.hpp"

namespace gui
{
namespace log
{
class NoLabelScaleDraw : public QwtScaleDraw
{
public:
  virtual QwtText label(double) const override
  {
    return QwtText("");  // 空のラベルを返す
  }
};

QwtPlot2::QwtPlot2(QWidget* parent) : super(parent)
{
  // 最小化可能に
  setMinimumSize(1, 1);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

  // 内部に凡例を配置
  const auto legend_item = new QwtPlotLegendItem();
  legend_item->attach(this);
  legend_item->setAlignment(Qt::AlignTop | Qt::AlignRight);  // 右上に配置
  legend_item->setMaxColumns(1);                             // 1列に並べる
  legend_item->setBackgroundMode(QwtPlotLegendItem::LegendBackground);
  legend_item->setBorderPen(QPen(Qt::black, 1));  // 黒の枠線

  // X軸のラベルを削除
  setAxisScaleDraw(QwtPlot::xBottom, new NoLabelScaleDraw());
}
}  // namespace log
}  // namespace gui
