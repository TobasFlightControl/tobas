// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rotors_viewer/speedmeter.hpp"

#include <algorithm>
#include <cmath>

#include <QEvent>
#include <QFontMetrics>
#include <QResizeEvent>
#include <QVBoxLayout>

#include <qwt/qwt_scale_draw.h>
#include <qwt/qwt_text.h>

#include <tobas_qt_tools/color.hpp>

namespace tobas::gui::ctrl
{
namespace
{
constexpr int kMaxPipeWidth = 50;
constexpr int kMinHeightForBottomText = 180;
constexpr int kGaugeBorderWidth = 2;
constexpr int kMinPipeWidthWithScale = 8;
constexpr int kNamePadding = 4;

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
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setAutoFillBackground(true);
  setBackgroundColor("transparent");

  top_text_ = new QLabel();
  top_text_->setAlignment(Qt::AlignCenter);
  top_text_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

  bottom_text_ = new qt::FramedLabel();
  bottom_text_->setAlignment(Qt::AlignCenter);
  bottom_text_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

  gauge_ = new QwtThermo();
  gauge_->setScaleDraw(new RpmScaleDraw());
  gauge_->setOrientation(Qt::Vertical);
  gauge_->setScale(0.0, 100.0);
  gauge_->setPipeWidth(kMaxPipeWidth);
  gauge_->setFillBrush(qt::color::cyan500());
  gauge_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
  gauge_->installEventFilter(this);

  gauge_row_ = new QHBoxLayout();
  gauge_row_->setContentsMargins(0, 0, 0, 0);
  gauge_row_->setSpacing(0);
  gauge_row_->addStretch();
  gauge_row_->addWidget(gauge_);
  gauge_row_->addStretch();
  updateGaugeAlignment();

  const auto rows = new QVBoxLayout(this);
  rows->setSizeConstraint(QLayout::SetNoConstraint);
  rows->setContentsMargins(0, 0, 0, 0);
  rows->setSpacing(2);
  rows->addWidget(top_text_, 0);
  rows->addLayout(gauge_row_, 1);
  rows->addWidget(bottom_text_, 0);
}

QSize SpeedmeterWidget::minimumSizeHint() const
{
  return { 0, 0 };
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
  return top_text_value_;
}

QString SpeedmeterWidget::getBottomText() const
{
  return bottom_text_value_;
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
  top_text_value_ = _text;
  updateLabels();
}

void SpeedmeterWidget::setBottomText(const QString& _text)
{
  bottom_text_value_ = _text;
  updateLabels();
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
  bottom_text_->setVisible(_event->size().height() >= kMinHeightForBottomText);
  updateGaugeAlignment();
}

void SpeedmeterWidget::updateGaugeValue()
{
  gauge_->setValue(value_);
}

void SpeedmeterWidget::updateGaugeAlignment()
{
  // Measure the tick labels and their gap from the pipe, even when the scale is currently hidden.
  const auto draw = static_cast<const QwtThermo*>(gauge_)->scaleDraw();
  const auto scale_width = static_cast<int>(std::ceil(draw->extent(gauge_->font()))) + gauge_->spacing();

  // Show the scale only if the assigned width can also hold both borders and a readable pipe.
  const auto show_scale = width() >= scale_width + 2 * kGaugeBorderWidth + kMinPipeWidthWithScale;
  const auto label_offset = show_scale ? scale_width : 0;

  // In very narrow cells, shrink the borders too so at least a one-pixel pipe can remain visible.
  const auto border_width = std::min(kGaugeBorderWidth, std::max(0, (width() - 1) / 2));

  gauge_->setScalePosition(show_scale ? QwtThermo::TrailingScale : QwtThermo::NoScale);
  gauge_->setBorderWidth(border_width);

  // Use the remaining width for the pipe, capped at its normal width when the window is wide enough.
  gauge_->setPipeWidth(std::clamp(width() - label_offset - 2 * border_width, 1, kMaxPipeWidth));

  // Refresh the layout and label text after changing the gauge geometry.
  gauge_row_->invalidate();
  updateLabels();
}

void SpeedmeterWidget::updateLabels()
{
  // Removing the pipe and borders from the gauge width leaves the scale's horizontal extent (zero if hidden).
  const auto scale_width = gauge_->minimumSizeHint().width() - gauge_->pipeWidth() - 2 * gauge_->borderWidth();
  const auto update_label = [this, scale_width](QLabel* _label, const QString& _text, int _padding)
  {
    const auto metrics = _label->fontMetrics();
    const auto available_width = std::max(0, width() - 2 * _padding);
    // A left inset equal to the scale width centers the text over the pipe.
    // Reduce that inset for long text so it can use more of the meter's width.
    const auto offset = std::clamp(available_width - metrics.horizontalAdvance(_text), 0, scale_width);
    _label->setContentsMargins(_padding + offset, 0, _padding, 0);
    // Shorten only the displayed text; retain the original strings for getters and tooltips.
    _label->setText(metrics.elidedText(_text, Qt::ElideMiddle, available_width - offset));
  };

  // Separate adjacent names without reducing the width available to the pipe itself.
  update_label(top_text_, top_text_value_, kNamePadding);
  update_label(bottom_text_, bottom_text_value_, 0);

  // The full name and RPM remain accessible when text is elided or the scale is hidden.
  const auto tooltip = top_text_value_ + '\n' + bottom_text_value_;
  setToolTip(tooltip);
  gauge_->setToolTip(tooltip);
}
}  // namespace tobas::gui::ctrl
