#include "tobas_flight_log_gui/log_viewer/plots/qwt_plot.hpp"

#include <qwt/qwt_plot_legenditem.h>
#include <qwt/qwt_scale_draw.h>

namespace gui
{
namespace log
{
namespace
{
class NoLabelScaleDraw : public QwtScaleDraw
{
public:
  virtual QwtText label(double) const override
  {
    return QwtText("");  // 空のラベルを返す
  }
};

class UnitScaleDraw : public QwtScaleDraw
{
public:
  explicit UnitScaleDraw(const QString& unit) : unit_(unit)
  {
  }

  QwtText label(double v) const override
  {
    return QwtText(QString::number(v) + " " + unit_);
  }

private:
  const QString unit_;
};

class BinaryScaleDraw : public QwtScaleDraw
{
  using super = QwtScaleDraw;

public:
  explicit BinaryScaleDraw(const QString& label0, const QString& label1) : label0_(label0), label1_(label1)
  {
  }

  QwtText label(double v) const override
  {
    if (v < 0.5) {
      return QwtText(label0_);
    }
    else {
      return QwtText(label1_);
    }
  }

private:
  const QString label0_, label1_;
};
}  // namespace

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
}

void QwtPlot2::setAxisNoLabel(const QwtPlot::Axis& axis)
{
  setAxisScaleDraw(axis, new NoLabelScaleDraw());
}

void QwtPlot2::setAxisLabelUnit(const QwtPlot::Axis& axis, const QString& unit)
{
  setAxisScaleDraw(axis, new UnitScaleDraw(unit));
}

void QwtPlot2::setupBinaryPlot(const QString& label0, const QString& label1)
{
  const QList<double> none;
  const QList<double> majors({ 0, 1 });
  const QwtScaleDiv ydiv(0, 1, none, none, majors);
  setAxisScaleDiv(QwtPlot::yLeft, ydiv);
  setAxisScaleDraw(QwtPlot::yLeft, new BinaryScaleDraw(label0, label1));
}
}  // namespace log
}  // namespace gui
