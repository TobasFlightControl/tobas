// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/utilities/qwt_plot.hpp"

#include <qwt/qwt_plot_legenditem.h>
#include <qwt/qwt_scale_div.h>
#include <qwt/qwt_scale_draw.h>
#include <qwt/qwt_scale_engine.h>
#include <qwt/qwt_text.h>

#include <algorithm>
#include <cmath>

#include <QFont>
#include <QFontInfo>
#include <QPen>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/utilities/constants.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
namespace
{
class LimitedLabelScaleEngine : public QwtLinearScaleEngine
{
  using super = QwtLinearScaleEngine;

public:
  QwtScaleDiv divideScale(double min, double max, int max_major, int max_minor, double step) const override
  {
    auto scale_div = super::divideScale(min, max, max_major, max_minor, step);
    auto ticks = scale_div.ticks(QwtScaleDiv::MajorTick);

    // Explicit step sizes and aligned limits can exceed Qwt's major-interval setting.
    while (ticks.size() > kMaxVerticalAxisLabels) {
      const auto tick_step = std::abs(ticks.at(1) - ticks.at(0));
      const auto next_step = tick_step > 0.0 ? tick_step * 2.0 : std::abs(max - min);
      scale_div = super::divideScale(min, max, max_major, max_minor, next_step);
      ticks = scale_div.ticks(QwtScaleDiv::MajorTick);
    }

    return scale_div;
  }
};

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

  // Use the auxiliary font size for all axis labels.
  for (int axis = 0; axis < QwtPlot::axisCnt; ++axis) {
    auto axis_font = axisFont(axis);
    axis_font.setPointSize(8);
    setAxisFont(axis, axis_font);
  }

  // Limit the number of major tick labels on both vertical axes.
  for (const auto axis : { QwtPlot::yLeft, QwtPlot::yRight }) {
    setAxisScaleEngine(axis, new LimitedLabelScaleEngine());
    setAxisMaxMajor(axis, kMaxVerticalScaleSteps);
  }

  // Place the legend inside.
  const auto legend_item = new QwtPlotLegendItem();
  legend_item->attach(this);
  legend_item->setAlignment(Qt::AlignTop | Qt::AlignRight);  // Place it at the upper right.
  legend_item->setMaxColumns(1);                             // Arrange in one column.
  legend_item->setBorderPen(QPen(Qt::black, 0.5));           // Draw a thin black border.
  legend_item->setFont(qt::DefaultFont(8));                  // Auxiliary font size.
  legend_item->setMargin(1);                                 // 1-pixel outer margin.
  legend_item->setItemMargin(1);                             // 1-pixel margin around each item.
  legend_item->setItemSpacing(1);                            // 1-pixel gap between icon and label.
  legend_item->setBorderDistance(2);                         // Qwt 6.1 equivalent of the canvas offset.
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
  const auto num_ticks = std::min<int>(size, kMaxVerticalAxisLabels);
  for (int i = 0; i < num_ticks; ++i) {
    // Retain the first and last states when thinning categorical labels.
    const auto index = num_ticks > 1 ? i * (size - 1) / (num_ticks - 1) : 0;
    majors.append(index);
  }

  const QwtScaleDiv ydiv(0, size - 1, none, none, majors);
  setAxisScaleDiv(QwtPlot::yLeft, ydiv);

  setAxisScaleDraw(QwtPlot::yLeft, new IndexedLabelScaleDraw(labels));
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
