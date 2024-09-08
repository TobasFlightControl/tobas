#pragma once

#include <QLabel>
#include <QLineEdit>

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
  void setValue(int value);

  void setText(const QString& text);
  void setMinimum(int minimum);
  void setMaximum(int maximum);
  void setSuffix(const QString& suffix);
  void setCenterValue();

private Q_SLOTS:
  void onSliderValueChanged(int value);

private:
  QString suffix_;

  QLabel* text_;
  QLineEdit* value_;
  Slider* slider_;
};

class DoubleSliderDisplay : public QWidget
{
  Q_OBJECT

  using self = DoubleSliderDisplay;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit DoubleSliderDisplay(int decimals = 6, QWidget* parent = nullptr);

  double getValue() const;
  void setValue(double value);

  void setText(const QString& text);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSuffix(const QString& suffix);
  void setCenterValue();

private Q_SLOTS:
  void onSliderValueChanged(double value);

private:
  const int decimals_;
  QString suffix_;

  QLabel* text_;
  QLineEdit* value_;
  DoubleSlider* slider_;
};
}  // namespace qt
