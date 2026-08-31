// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/wait_spinner.hpp"

#include <algorithm>

#include <QPainter>

namespace tobas
{
namespace qt
{
namespace
{
int lineCountDistanceFromPrimary(int current, int primary, int total_num_lines)
{
  auto distance = primary - current;
  if (distance < 0) {
    distance += total_num_lines;
  }
  return distance;
}

QColor currentLineColor(int count_dist, int total_num_lines, double trail_fade_perc, double min_opacity, QColor color)
{
  if (count_dist == 0) {
    return color;
  }

  const auto min_alpha = min_opacity / 100.0;
  const auto dist_thresh = static_cast<int>(std::ceil((total_num_lines - 1) * trail_fade_perc / 100.0));

  if (count_dist > dist_thresh) {
    color.setAlphaF(min_alpha);
  }
  else {
    const auto alpha_diff = color.alphaF() - min_alpha;
    const auto gradient = alpha_diff / static_cast<double>(dist_thresh + 1);
    const auto result_alpha = std::clamp(color.alphaF() - gradient * count_dist, 0.0, 1.0);
    color.setAlphaF(result_alpha);
  }

  return color;
}
}  // namespace

WaitSpinnerWidget::WaitSpinnerWidget(QWidget* parent, bool center_on_parent)
  : super(parent), center_on_parent_(center_on_parent)
{
  initialize();
}

WaitSpinnerWidget::WaitSpinnerWidget(Qt::WindowModality modality, QWidget* parent, bool center_on_parent)
  : super(parent, Qt::Dialog | Qt::FramelessWindowHint), center_on_parent_(center_on_parent)
{
  initialize();

  // We need to set the window modality AFTER we've hidden the widget for the first time
  // since changing this property while the widget is visible has no effect.
  setWindowModality(modality);
  setAttribute(Qt::WA_TranslucentBackground);
}

void WaitSpinnerWidget::initialize()
{
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &self::rotate);
  updateSize();
  updateTimer();
  hide();
}

void WaitSpinnerWidget::paintEvent(QPaintEvent*)
{
  updatePosition();

  QPainter painter(this);
  painter.fillRect(rect(), Qt::transparent);
  painter.setRenderHint(QPainter::Antialiasing, true);

  if (cur_counter_ >= num_lines_) {
    cur_counter_ = 0;
  }

  painter.setPen(Qt::NoPen);
  for (int i = 0; i < num_lines_; ++i) {
    painter.save();
    painter.translate(inner_radius_ + line_length_, inner_radius_ + line_length_);
    const auto rotate_angle = static_cast<double>(360 * i) / static_cast<double>(num_lines_);
    painter.rotate(rotate_angle);
    painter.translate(inner_radius_, 0);
    const auto distance = lineCountDistanceFromPrimary(i, cur_counter_, num_lines_);
    const auto color = currentLineColor(distance, num_lines_, trail_fade_perc_, min_trail_opacity_, color_);
    painter.setBrush(color);
    // TODO: Improve the way rounded rect is painted.
    painter.drawRoundedRect(
      QRect(0, -line_width_ / 2, line_length_, line_width_), roundness_, roundness_, Qt::RelativeSize);
    painter.restore();
  }
}

void WaitSpinnerWidget::start()
{
  if (is_spinning_) {
    return;
  }

  updatePosition();
  show();

  timer_->start();
  cur_counter_ = 0;
  is_spinning_ = true;
}

void WaitSpinnerWidget::stop()
{
  if (!is_spinning_) {
    return;
  }

  hide();

  timer_->stop();
  cur_counter_ = 0;
  is_spinning_ = false;
}

void WaitSpinnerWidget::setColor(QColor color)
{
  color_ = color;
}

void WaitSpinnerWidget::setRoundness(double roundness)
{
  roundness_ = std::max(0.0, std::min(100.0, roundness));
}

void WaitSpinnerWidget::setMinimumTrailOpacity(double min_trail_opacity)
{
  min_trail_opacity_ = min_trail_opacity;
}

void WaitSpinnerWidget::setTrailFadePercentage(double trail)
{
  trail_fade_perc_ = trail;
}

void WaitSpinnerWidget::setRevolutionsPerSecond(double revolution_per_sec)
{
  revolutions_per_sec_ = revolution_per_sec;
  updateTimer();
}

void WaitSpinnerWidget::setNumberOfLines(int lines)
{
  num_lines_ = lines;
  cur_counter_ = 0;
  updateTimer();
}

void WaitSpinnerWidget::setLineLength(int length)
{
  line_length_ = length;
  updateSize();
}

void WaitSpinnerWidget::setLineWidth(int width)
{
  line_width_ = width;
  updateSize();
}

void WaitSpinnerWidget::setInnerRadius(int radius)
{
  inner_radius_ = radius;
  updateSize();
}

QColor WaitSpinnerWidget::color() const
{
  return color_;
}

double WaitSpinnerWidget::roundness() const
{
  return roundness_;
}

double WaitSpinnerWidget::minimumTrailOpacity() const
{
  return min_trail_opacity_;
}

double WaitSpinnerWidget::trailFadePercentage() const
{
  return trail_fade_perc_;
}

double WaitSpinnerWidget::revolutionsPersSecond() const
{
  return revolutions_per_sec_;
}

int WaitSpinnerWidget::numberOfLines() const
{
  return num_lines_;
}

int WaitSpinnerWidget::lineLength() const
{
  return line_length_;
}

int WaitSpinnerWidget::lineWidth() const
{
  return line_width_;
}

int WaitSpinnerWidget::innerRadius() const
{
  return inner_radius_;
}

bool WaitSpinnerWidget::isSpinning() const
{
  return is_spinning_;
}

void WaitSpinnerWidget::rotate()
{
  ++cur_counter_;
  if (cur_counter_ >= num_lines_) {
    cur_counter_ = 0;
  }
  update();
}

void WaitSpinnerWidget::updateSize()
{
  int size = (inner_radius_ + line_length_) * 2;
  setFixedSize(size, size);
}

void WaitSpinnerWidget::updateTimer()
{
  timer_->setInterval(1000 / (num_lines_ * revolutions_per_sec_));
}

void WaitSpinnerWidget::updatePosition()
{
  if (parentWidget() && center_on_parent_) {
    move(parentWidget()->width() / 2 - width() / 2, parentWidget()->height() / 2 - height() / 2);
  }
}
}  // namespace qt
}  // namespace tobas
