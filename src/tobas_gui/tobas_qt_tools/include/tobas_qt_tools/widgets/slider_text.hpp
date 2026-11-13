// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>

#include "./double_slider.hpp"
#include "./slider.hpp"

namespace tobas::qt
{
class IntSliderTextWidget : public QWidget
{
  Q_OBJECT

  using self = IntSliderTextWidget;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(int _value);

public:
  explicit IntSliderTextWidget(int _minimum, int _maximum, QWidget* _parent = nullptr);

  int get() const;
  void set(int _value);

private:
  Slider* slider_;
  QLineEdit* line_edit_;

  void setSliderValue(int _value);
  void setLineEditText(int _value);

private Q_SLOTS:
  void onSliderValueChanged();
  void onLineEditReturnPressed();
};

class DoubleSliderTextWidget : public QWidget
{
  Q_OBJECT

  using self = DoubleSliderTextWidget;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double _value);

public:
  explicit DoubleSliderTextWidget(double _minimum, double _maximum, int _decimals = 6, QWidget* _parent = nullptr);

  double get() const;
  void set(double _value);

private:
  const int decimals_;
  DoubleSlider* slider_;
  QLineEdit* line_edit_;

  void setSliderValue(double _value);
  void setLineEditText(double _value);

private Q_SLOTS:
  void onSliderValueChanged();
  void onLineEditReturnPressed();
};
}  // namespace tobas::qt
