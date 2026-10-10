// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QLineEdit>

#include "./double_slider.hpp"
#include "./slider.hpp"

namespace tobas
{
namespace qt
{
class IntSliderDisplay : public QWidget
{
  Q_OBJECT

  using self = IntSliderDisplay;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(int _value);

public:
  explicit IntSliderDisplay(QWidget* _parent = nullptr);

  int getValue() const;
  int getMinimum() const;
  int getMaximum() const;
  QString getText() const;
  QString getSuffix() const;

  void setValue(int _value, bool _block_signal = false);
  void setMinimum(int _minimum);
  void setMaximum(int _maximum);
  void setRange(int _minimum, int _maximum);
  void setText(const QString& _text);
  void setSuffix(const QString& _suffix);

private:
  QString suffix_;

  QLabel* text_;
  QLineEdit* value_;
  Slider* slider_;

  void updateValueText(int _value);

private Q_SLOTS:
  void onSliderValueChanged(int _value);
};

class DoubleSliderDisplay : public QWidget
{
  Q_OBJECT

  using self = DoubleSliderDisplay;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double _value);

public:
  explicit DoubleSliderDisplay(QWidget* _parent = nullptr);

  double getValue() const;
  double getMinimum() const;
  double getMaximum() const;
  QString getText() const;
  QString getSuffix() const;
  int getDecimals() const;

  void setValue(double _value, bool _block_signal = false);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setRange(double _minimum, double _maximum);
  void setText(const QString& _text);
  void setSuffix(const QString& _suffix);
  void setDecimals(int _decimals);

private:
  QString suffix_;
  int decimals_ = 6;

  QLabel* text_;
  QLineEdit* value_;
  DoubleSlider* slider_;

  void updateValueText(double _value);

private Q_SLOTS:
  void onSliderValueChanged(double _value);
};
}  // namespace qt
}  // namespace tobas
