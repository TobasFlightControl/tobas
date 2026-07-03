// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/qwt_plot.hpp"

#include <qwt/qwt_plot_legenditem.h>
#include <qwt/qwt_scale_draw.h>

namespace tobas
{
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
    return QwtText("");  // Return an empty label.
  }
};

class UnitScaleDraw : public QwtScaleDraw
{
public:
  explicit UnitScaleDraw(const QString& unit) : unit_(unit)
  {
  }

  QwtText label(double value) const override
  {
    return QwtText(QString::number(value) + " " + unit_);
  }

private:
  const QString unit_;
};

class IndexedLabelScaleDraw : public QwtScaleDraw
{
  using super = QwtScaleDraw;

public:
  explicit IndexedLabelScaleDraw(const QStringList& labels) : labels_(labels)
  {
  }

  QwtText label(double value) const override
  {
    const auto idx = qRound(value);
    if (idx < 0 || labels_.size() <= idx) {
      return {};
    }
    return QwtText(labels_.at(idx));
  }

private:
  const QStringList labels_;
};
}  // namespace

QwtPlot2::QwtPlot2(QWidget* parent) : super(parent)
{
  // Make it minimizable.
  setMinimumSize(1, 1);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

  // Place the legend inside.
  const auto legend_item = new QwtPlotLegendItem();
  legend_item->attach(this);
  legend_item->setAlignment(Qt::AlignTop | Qt::AlignRight);  // Place it at the upper right.
  legend_item->setMaxColumns(1);                             // Arrange in one column.
  legend_item->setBackgroundMode(QwtPlotLegendItem::LegendBackground);
  legend_item->setBorderPen(QPen(Qt::black, 1));  // Black border.
}

void QwtPlot2::setAxisNoLabel(const QwtPlot::Axis& axis)
{
  setAxisScaleDraw(axis, new NoLabelScaleDraw());
}

void QwtPlot2::setAxisLabelUnit(const QwtPlot::Axis& axis, const QString& unit)
{
  setAxisScaleDraw(axis, new UnitScaleDraw(unit));
}

void QwtPlot2::setupIndexedLabelPlot(const QStringList& labels)
{
  const auto size = labels.size();

  const QList<double> none;

  QList<double> majors;
  for (int i = 0; i < size; ++i) {
    majors.append(i);
  }

  const QwtScaleDiv ydiv(0, size - 1, none, none, majors);
  setAxisScaleDiv(QwtPlot::yLeft, ydiv);

  setAxisScaleDraw(QwtPlot::yLeft, new IndexedLabelScaleDraw(labels));
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
