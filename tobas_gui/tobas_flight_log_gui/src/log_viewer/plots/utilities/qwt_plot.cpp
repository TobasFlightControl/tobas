// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/utilities/qwt_plot.hpp"

#include <qwt/qwt_plot_legenditem.h>
#include <qwt/qwt_scale_draw.h>

namespace tobas::gui::log
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
  explicit UnitScaleDraw(const QString& _unit) : unit_(_unit)
  {
  }

  QwtText label(double _value) const override
  {
    return QwtText(QString::number(_value) + " " + unit_);
  }

private:
  const QString unit_;
};

class IndexedLabelScaleDraw : public QwtScaleDraw
{
  using super = QwtScaleDraw;

public:
  explicit IndexedLabelScaleDraw(const QStringList& _labels) : labels_(_labels)
  {
  }

  QwtText label(double _value) const override
  {
    const auto idx = qRound(_value);
    if (idx < 0 || labels_.size() <= idx) {
      return {};
    }
    return QwtText(labels_.at(idx));
  }

private:
  const QStringList labels_;
};
}  // namespace

QwtPlot2::QwtPlot2(QWidget* _parent) : super(_parent)
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

void QwtPlot2::setAxisNoLabel(QwtPlot::Axis _axis)
{
  setAxisScaleDraw(_axis, new NoLabelScaleDraw());
}

void QwtPlot2::setAxisLabelUnit(QwtPlot::Axis _axis, const QString& _unit)
{
  setAxisScaleDraw(_axis, new UnitScaleDraw(_unit));
}

void QwtPlot2::setupIndexedLabelPlot(const QStringList& _labels)
{
  const auto size = _labels.size();

  const QList<double> none;

  QList<double> majors;
  for (int i = 0; i < size; ++i) {
    majors.append(i);
  }

  const QwtScaleDiv ydiv(0, size - 1, none, none, majors);
  setAxisScaleDiv(QwtPlot::yLeft, ydiv);

  setAxisScaleDraw(QwtPlot::yLeft, new IndexedLabelScaleDraw(_labels));
}
}  // namespace tobas::gui::log
