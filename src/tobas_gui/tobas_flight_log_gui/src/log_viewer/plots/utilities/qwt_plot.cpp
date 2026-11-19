// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/log_viewer/plots/utilities/qwt_plot.hpp"

#include <algorithm>
#include <cmath>

#include <QFont>
#include <QFontInfo>
#include <QPen>

#include <qwt/qwt_plot_legenditem.h>
#include <qwt/qwt_scale_div.h>
#include <qwt/qwt_scale_draw.h>
#include <qwt/qwt_scale_engine.h>
#include <qwt/qwt_text.h>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/utilities/constants.hpp"

namespace tobas::gui::log
{
namespace
{
class LimitedLabelScaleEngine : public QwtLinearScaleEngine
{
  using super = QwtLinearScaleEngine;

public:
  QwtScaleDiv divideScale(double _min, double _max, int _max_major, int _max_minor, double _step) const override
  {
    auto scale_div = super::divideScale(_min, _max, _max_major, _max_minor, _step);
    auto ticks = scale_div.ticks(QwtScaleDiv::MajorTick);

    // Explicit step sizes and aligned limits can exceed Qwt's major-interval setting.
    while (ticks.size() > kMaxVerticalAxisLabels) {
      const auto step = std::abs(ticks.at(1) - ticks.at(0));
      const auto next_step = step > 0.0 ? step * 2.0 : std::abs(_max - _min);
      scale_div = super::divideScale(_min, _max, _max_major, _max_minor, next_step);
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

  // Use the auxiliary font size for all axis labels.
  for (int axis = 0; axis < QwtPlot::axisCnt; ++axis) {
    auto axis_font = axisFont(axis);
    axis_font.setPointSize(cmn::kAuxiliaryPointSize);
    setAxisFont(axis, axis_font);
  }

  // Limit the number of major tick labels on both vertical axes.
  for (const auto axis : { QwtPlot::yLeft, QwtPlot::yRight }) {
    setAxisScaleEngine(axis, new LimitedLabelScaleEngine());
    setAxisMaxMajor(axis, kMaxVerticalScaleSteps);
  }

  // Place the legend inside.
  legend_item_ = new QwtPlotLegendItem();
  legend_item_->attach(this);
  legend_item_->setAlignmentInCanvas(Qt::AlignTop | Qt::AlignRight);  // Place it at the upper right.
  legend_item_->setMaxColumns(1);                                     // Arrange in one column.
  legend_item_->setBorderPen(QPen(Qt::black, 0.5));                   // Draw a thin black border.
  legend_item_->setFont(qt::DefaultFont(cmn::kAuxiliaryPointSize));   // Auxiliary font size.
  legend_item_->setMargin(1);                                         // 1-pixel outer margin.
  legend_item_->setItemMargin(1);                                     // 1-pixel margin around each item.
  legend_item_->setItemSpacing(1);                                    // 1-pixel gap between icon and label.
  legend_item_->setOffsetInCanvas(Qt::Horizontal | Qt::Vertical, 2);  // 2-pixel inset from canvas edges.
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
  const auto num_ticks = std::min<qsizetype>(size, kMaxVerticalAxisLabels);
  for (int i = 0; i < num_ticks; ++i) {
    // Retain the first and last states when thinning categorical labels.
    const auto index = num_ticks > 1 ? i * (size - 1) / (num_ticks - 1) : 0;
    majors.append(index);
  }

  const QwtScaleDiv ydiv(0, size - 1, none, none, majors);
  setAxisScaleDiv(QwtPlot::yLeft, ydiv);

  setAxisScaleDraw(QwtPlot::yLeft, new IndexedLabelScaleDraw(_labels));
}
}  // namespace tobas::gui::log
