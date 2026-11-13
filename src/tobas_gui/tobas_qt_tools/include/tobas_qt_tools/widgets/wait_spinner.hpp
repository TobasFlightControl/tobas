// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

#include <QColor>
#include <QTimer>
#include <QWidget>

namespace tobas::qt
{
class WaitSpinnerWidget : public QWidget
{
  Q_OBJECT

  using self = WaitSpinnerWidget;
  using super = QWidget;

public:
  /**
   * Constructor for "standard" widget behaviour.
   * Use this constructor if you wish to, e.g. embed your widget in another.
   */
  explicit WaitSpinnerWidget(QWidget* _parent = nullptr, bool _center_on_parent = true);

  /**
   * Constructor.
   * Use this constructor to automatically create a modal ("blocking") spinner on top of the calling widget/window.
   * If a valid parent widget is provided, `center_on_parent` will ensure
   * that the spinner automatically centers itself on it, if not, `center_on_parent` is ignored.
   */
  explicit WaitSpinnerWidget(Qt::WindowModality _modality, QWidget* _parent = nullptr, bool _center_on_parent = true);

  void setColor(QColor _color);
  void setRoundness(double _roundness);
  void setMinimumTrailOpacity(double _min_trail_opacity);
  void setTrailFadePercentage(double _trail);
  void setRevolutionsPerSecond(double _revolution_per_sec);
  void setNumberOfLines(int _lines);
  void setLineLength(int _length);
  void setLineWidth(int _width);
  void setInnerRadius(int _radius);

  QColor color() const;
  double roundness() const;
  double minimumTrailOpacity() const;
  double trailFadePercentage() const;
  double revolutionsPersSecond() const;
  int numberOfLines() const;
  int lineLength() const;
  int lineWidth() const;
  int innerRadius() const;

  bool isSpinning() const;

public Q_SLOTS:
  void start();
  void stop();

protected:
  void paintEvent(QPaintEvent* _event);

private Q_SLOTS:
  void rotate();

private:
  const bool center_on_parent_;

  QColor color_ = Qt::black;
  double roundness_ = 100.0;  ///< 0..100
  double min_trail_opacity_ = M_PI;
  double trail_fade_perc_ = 80.0;
  double revolutions_per_sec_ = M_PI_2;
  int num_lines_ = 20;
  int line_length_ = 10;
  int line_width_ = 2;
  int inner_radius_ = 10;
  int cur_counter_ = 0;
  bool is_spinning_ = false;

  QTimer* timer_;

  void initialize();
  void updateSize();
  void updateTimer();
  void updatePosition();
};
}  // namespace tobas::qt
