#pragma once

#include <QLabel>
#include <QLineEdit>

#include "./double_slider.hpp"
#include "./slider.hpp"

namespace qt
{
class IntSliderDisplay : public QWidget
{
  Q_OBJECT

  using self = IntSliderDisplay;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(int value);

public:
  explicit IntSliderDisplay(QWidget* parent = nullptr);

  int getValue() const;
  int getMinimum() const;
  int getMaximum() const;
  QString getText() const;
  QString getSuffix() const;

  void setValue(int value, bool block_signal = false);
  void setMinimum(int minimum);
  void setMaximum(int maximum);
  void setRange(int minimum, int maximum);
  void setText(const QString& text);
  void setSuffix(const QString& suffix);

private:
  QString suffix_;

  QLabel* text_;
  QLineEdit* value_;
  Slider* slider_;

  void updateValueText(int value);

private Q_SLOTS:
  void onSliderValueChanged(int value);
};

class DoubleSliderDisplay : public QWidget
{
  Q_OBJECT

  using self = DoubleSliderDisplay;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit DoubleSliderDisplay(QWidget* parent = nullptr);

  double getValue() const;
  double getMinimum() const;
  double getMaximum() const;
  QString getText() const;
  QString getSuffix() const;
  int getDecimals() const;

  void setValue(double value, bool block_signal = false);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setRange(double minimum, double maximum);
  void setText(const QString& text);
  void setSuffix(const QString& suffix);
  void setDecimals(int decimals);

private:
  QString suffix_;
  int decimals_ = 6;

  QLabel* text_;
  QLineEdit* value_;
  DoubleSlider* slider_;

  void updateValueText(double value);

private Q_SLOTS:
  void onSliderValueChanged(double value);
};
}  // namespace qt
