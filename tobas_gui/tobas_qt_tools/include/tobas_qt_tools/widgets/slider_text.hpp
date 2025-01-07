#pragma once

#include <QLabel>
#include <QLineEdit>

#include "./slider.hpp"
#include "./double_slider.hpp"

namespace qt
{
class IntSliderTextWidget : public QWidget
{
  Q_OBJECT

  using self = IntSliderTextWidget;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(int value);

public:
  explicit IntSliderTextWidget(int minimum, int maximum, QWidget* parent = nullptr);

  int get() const;
  void set(int value);

private Q_SLOTS:
  void onSliderReleased();
  void onLineEditReturnPressed();

private:
  Slider* slider_;
  QLineEdit* line_edit_;

  void setSliderValue(int value);
  void setLineEditText(int value);
};

class DoubleSliderTextWidget : public QWidget
{
  Q_OBJECT

  using self = DoubleSliderTextWidget;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(double value);

public:
  explicit DoubleSliderTextWidget(double minimum, double maximum, int decimals = 6, QWidget* parent = nullptr);

  double get() const;
  void set(double value);

private Q_SLOTS:
  void onSliderReleased();
  void onLineEditReturnPressed();

private:
  const int decimals_;
  DoubleSlider* slider_;
  QLineEdit* line_edit_;

  void setSliderValue(double value);
  void setLineEditText(double value);
};
}  // namespace qt
