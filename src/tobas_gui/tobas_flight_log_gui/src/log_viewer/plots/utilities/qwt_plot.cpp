// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/utilities/qwt_plot.hpp"

#include <QFont>
#include <QFontInfo>
#include <QPen>

#include <qwt/qwt_plot_legenditem.h>
#include <qwt/qwt_scale_draw.h>
#include <qwt/qwt_text.h>

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
  legend_item_ = new QwtPlotLegendItem();
  legend_item_->attach(this);
  legend_item_->setAlignmentInCanvas(Qt::AlignTop | Qt::AlignRight);  // Place it at the upper right.
  legend_item_->setMaxColumns(1);                                     // Arrange in one column.
  legend_item_->setBackgroundMode(QwtPlotLegendItem::LegendBackground);
  legend_item_->setBorderPen(QPen(Qt::black, 1));  // Black border.
  setCompactLegend(false);
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

void QwtPlot2::resizeEvent(QResizeEvent* _event)
{
  super::resizeEvent(_event);

  // Base the legend size on each plot, including when its panel is expanded or restored.
  constexpr int kCompactLegendWidth = 400;
  constexpr int kCompactLegendHeight = 180;
  const auto compact = width() < kCompactLegendWidth || height() < kCompactLegendHeight;
  if (compact != compact_legend_) {
    setCompactLegend(compact);
    replot();
  }
}

void QwtPlot2::setCompactLegend(bool _compact)
{
  compact_legend_ = _compact;

  auto legend_font = font();
  if (_compact) {
    legend_font.setPointSizeF(fontInfo().pointSizeF() * 0.6);
  }
  legend_item_->setFont(legend_font);
  legend_item_->setMargin(_compact ? 1 : 0);
  legend_item_->setItemMargin(_compact ? 1 : 4);
  legend_item_->setItemSpacing(_compact ? 2 : 4);
  legend_item_->setOffsetInCanvas(Qt::Horizontal | Qt::Vertical, _compact ? 2 : 10);
}
}  // namespace tobas::gui::log
