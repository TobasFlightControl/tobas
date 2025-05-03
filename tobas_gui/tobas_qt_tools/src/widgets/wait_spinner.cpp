#include <algorithm>
#include <QPainter>

#include "tobas_qt_tools/widgets/wait_spinner.hpp"

using namespace std;

namespace qt
{
WaitSpinnerWidget::WaitSpinnerWidget(QWidget* parent, bool center_on_parent, bool disable_parent_when_spinning)
  : QWidget(parent), center_on_parent_(center_on_parent), disable_parent_when_spinning_(disable_parent_when_spinning)
{
  initialize();
}

WaitSpinnerWidget::WaitSpinnerWidget(
  Qt::WindowModality modality,
  QWidget* parent,
  bool center_on_parent,
  bool disable_parent_when_spinning)
  : QWidget(parent, Qt::Dialog | Qt::FramelessWindowHint),
    center_on_parent_(center_on_parent),
    disable_parent_when_spinning_(disable_parent_when_spinning)
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
  painter.fillRect(this->rect(), Qt::transparent);
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
    // TODO improve the way rounded rect is painted
    painter.drawRoundedRect(
      QRect(0, -line_width_ / 2, line_length_, line_width_), roundness_, roundness_, Qt::RelativeSize);
    painter.restore();
  }
}

void WaitSpinnerWidget::start()
{
  updatePosition();
  is_spinning_ = true;
  show();

  if (parentWidget() && disable_parent_when_spinning_) {
    parentWidget()->setEnabled(false);
  }

  if (!timer_->isActive()) {
    timer_->start();
    cur_counter_ = 0;
  }
}

void WaitSpinnerWidget::stop()
{
  is_spinning_ = false;
  hide();

  if (parentWidget() && disable_parent_when_spinning_) {
    parentWidget()->setEnabled(true);
  }

  if (timer_->isActive()) {
    timer_->stop();
    cur_counter_ = 0;
  }
}

void WaitSpinnerWidget::setColor(QColor color)
{
  color_ = color;
}

void WaitSpinnerWidget::setRoundness(double roundness)
{
  roundness_ = max(0., min(100., roundness));
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

int WaitSpinnerWidget::lineCountDistanceFromPrimary(int current, int primary, int total_num_lines)
{
  auto distance = primary - current;
  if (distance < 0) {
    distance += total_num_lines;
  }
  return distance;
}

QColor WaitSpinnerWidget::currentLineColor(
  int count_dist,
  int total_num_lines,
  double trail_fade_perc,
  double min_opacity,
  QColor color)
{
  if (count_dist == 0) {
    return color;
  }

  const auto min_alpha_f = min_opacity / 100.;
  const auto dist_thresh = static_cast<int>(ceil((total_num_lines - 1) * trail_fade_perc / 100.));

  if (count_dist > dist_thresh) {
    color.setAlphaF(min_alpha_f);
  }
  else {
    const auto alpha_diff = color.alphaF() - min_alpha_f;
    const auto gradient = alpha_diff / static_cast<double>(dist_thresh + 1);
    auto result_alpha = color.alphaF() - gradient * count_dist;

    // If alpha is out of bounds, clip it.
    result_alpha = min(1., max(0., result_alpha));
    color.setAlphaF(result_alpha);
  }

  return color;
}
}  // namespace qt
