// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rotors_viewer/speedmeter.hpp"

#include <algorithm>
#include <cmath>

#include <QEvent>
#include <QResizeEvent>
#include <QVBoxLayout>

#include <qwt/qwt_scale_draw.h>
#include <qwt/qwt_text.h>

#include <tobas_qt_tools/color.hpp>

namespace tobas::gui::ctrl
{
namespace
{
class RpmScaleDraw : public QwtScaleDraw
{
public:
  QwtText label(double _value) const override
  {
    if (std::abs(_value) >= 1000.0) {
      return QwtText(QString::number(_value / 1000.0, 'g', 6) + "k");
    }
    return QwtScaleDraw::label(_value);
  }
};
}  // namespace

SpeedmeterWidget::SpeedmeterWidget()
{
  setAutoFillBackground(true);
  setBackgroundColor("transparent");

  top_text_ = new QLabel();
  top_text_->setAlignment(Qt::AlignCenter);

  bottom_text_ = new qt::FramedLabel();
  bottom_text_->setAlignment(Qt::AlignCenter);

  gauge_ = new QwtThermo();
  gauge_->setScaleDraw(new RpmScaleDraw());
  gauge_->setOrientation(Qt::Vertical);
  gauge_->setScale(0.0, 100.0);
  gauge_->setPipeWidth(50);
  gauge_->setFillBrush(qt::color::cyan500());
  gauge_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
  gauge_->installEventFilter(this);

  scale_spacer_ = new QSpacerItem(0, 0, QSizePolicy::Fixed, QSizePolicy::Minimum);
  gauge_row_ = new QHBoxLayout();
  gauge_row_->setContentsMargins(0, 0, 0, 0);
  gauge_row_->setSpacing(0);
  gauge_row_->addStretch();
  gauge_row_->addWidget(gauge_);
  gauge_row_->addItem(scale_spacer_);
  gauge_row_->addStretch();
  updateGaugeAlignment();

  const auto rows = new QVBoxLayout(this);
  rows->setContentsMargins(0, 0, 0, 0);
  rows->setSpacing(2);
  rows->addWidget(top_text_, 0);
  rows->addLayout(gauge_row_, 1);
  rows->addWidget(bottom_text_, 0);
}

QString SpeedmeterWidget::getBackgroundColor() const
{
  return palette().color(QPalette::Window).name(QColor::HexArgb);
}

double SpeedmeterWidget::getMaximumValue() const
{
  return gauge_->upperBound();
}

double SpeedmeterWidget::getMinimumValue() const
{
  return gauge_->lowerBound();
}

double SpeedmeterWidget::getValue() const
{
  return value_;
}

QString SpeedmeterWidget::getTopText() const
{
  return top_text_->text();
}

QString SpeedmeterWidget::getBottomText() const
{
  return bottom_text_->text();
}

void SpeedmeterWidget::setBackgroundColor(const QString& _text)
{
  auto background_palette = palette();
  background_palette.setColor(QPalette::Window, QColor(_text));
  setPalette(background_palette);
}

void SpeedmeterWidget::setMaximumValue(double _value)
{
  gauge_->setUpperBound(_value);
  updateGaugeAlignment();
}

void SpeedmeterWidget::setMinimumValue(double _value)
{
  gauge_->setLowerBound(_value);
  updateGaugeAlignment();
  updateGaugeValue();
}

void SpeedmeterWidget::setValue(double _value)
{
  value_ = _value;
  updateGaugeValue();
}

void SpeedmeterWidget::setTopText(const QString& _text)
{
  top_text_->setText(_text);
  updateGaugeAlignment();
}

void SpeedmeterWidget::setBottomText(const QString& _text)
{
  bottom_text_->setText(_text);
  updateGaugeAlignment();
}

bool SpeedmeterWidget::eventFilter(QObject* _watched, QEvent* _event)
{
  if (_watched == gauge_ && (_event->type() == QEvent::FontChange || _event->type() == QEvent::StyleChange)) {
    updateGaugeAlignment();
  }
  return QWidget::eventFilter(_watched, _event);
}

void SpeedmeterWidget::resizeEvent(QResizeEvent* _event)
{
  QWidget::resizeEvent(_event);

  // Reserve the available height for the gauge when the meter is compact.
  constexpr int kMinHeightForBottomText = 180;
  bottom_text_->setVisible(_event->size().height() >= kMinHeightForBottomText);
  updateGaugeAlignment();
}

void SpeedmeterWidget::updateGaugeAlignment()
{
  const auto scale_width = gauge_->minimumSizeHint().width() - gauge_->pipeWidth() - 2 * gauge_->borderWidth();

  top_text_->setContentsMargins(0, 0, 0, 0);
  bottom_text_->setContentsMargins(0, 0, 0, 0);
  const auto label_width = std::max(
    top_text_->minimumSizeHint().width(), bottom_text_->isHidden() ? 0 : bottom_text_->minimumSizeHint().width());
  const auto pipe_width = gauge_->pipeWidth() + 2 * gauge_->borderWidth();

  // Share alignment space between the labels and the gauge to minimize the meter width.
  const auto spacer_width = std::clamp((label_width - pipe_width + 1) / 2, 0, scale_width);
  const auto label_offset = scale_width - spacer_width;
  top_text_->setContentsMargins(label_offset, 0, 0, 0);
  bottom_text_->setContentsMargins(label_offset, 0, 0, 0);
  scale_spacer_->changeSize(spacer_width, 0, QSizePolicy::Fixed, QSizePolicy::Minimum);
  gauge_row_->invalidate();
}

void SpeedmeterWidget::updateGaugeValue()
{
  gauge_->setValue(value_);
}
}  // namespace tobas::gui::ctrl
