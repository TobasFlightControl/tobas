#include <QVBoxLayout>
#include <QHBoxLayout>

#include "tobas_qt_tools/widgets/slider_display.hpp"
#include "tobas_qt_tools/font.hpp"

#define TEXT_PSIZE 9

namespace qt
{
IntSliderDisplay::IntSliderDisplay(QWidget* parent) : super(parent)
{
  DefaultFont font(TEXT_PSIZE, QFont::Bold);

  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  text_ = new QLabel();
  text_->setFont(font);
  cols->addWidget(text_);

  value_ = new QLineEdit();
  value_->setAlignment(Qt::AlignRight);
  value_->setFont(font);
  value_->setReadOnly(true);
  value_->setFocusPolicy(Qt::NoFocus);
  cols->addWidget(value_);

  slider_ = new Slider(Qt::Horizontal);
  rows->addWidget(slider_);

  connect(slider_, &Slider::valueChanged, this, &self::onSliderValueChanged);
}

int IntSliderDisplay::getValue() const
{
  return slider_->value();
}

void IntSliderDisplay::setValue(int value, bool block_signal)
{
  slider_->blockSignals(true);
  slider_->setValue(value);
  slider_->blockSignals(false);

  value_->setText(QString::number(value) + suffix_);

  if (!block_signal)
    Q_EMIT valueChanged(value);
}

void IntSliderDisplay::setText(const QString& text)
{
  text_->setText(text);
}

void IntSliderDisplay::setMinimum(int minimum)
{
  slider_->setMinimum(minimum);
}

void IntSliderDisplay::setMaximum(int maximum)
{
  slider_->setMaximum(maximum);
}

void IntSliderDisplay::setSuffix(const QString& suffix)
{
  suffix_ = suffix;
  value_->setText(QString::number(getValue()) + suffix_);
}

void IntSliderDisplay::setCenterValue(bool block_signal)
{
  const auto value = (slider_->minimum() + slider_->maximum()) / 2;
  setValue(value, block_signal);
}

void IntSliderDisplay::onSliderValueChanged(int value)
{
  value_->setText(QString::number(value) + suffix_);
  Q_EMIT valueChanged(value);
}

DoubleSliderDisplay::DoubleSliderDisplay(int decimals, QWidget* parent) : super(parent), decimals_(decimals)
{
  DefaultFont font(TEXT_PSIZE, QFont::Bold);

  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  text_ = new QLabel();
  text_->setFont(font);
  cols->addWidget(text_);

  value_ = new QLineEdit();
  value_->setAlignment(Qt::AlignRight);
  value_->setFont(font);
  value_->setReadOnly(true);
  value_->setFocusPolicy(Qt::NoFocus);
  cols->addWidget(value_);

  slider_ = new DoubleSlider(Qt::Horizontal);
  rows->addWidget(slider_);

  connect(slider_, &DoubleSlider::valueChanged, this, &self::onSliderValueChanged);
}

double DoubleSliderDisplay::getValue() const
{
  return slider_->value();
}

void DoubleSliderDisplay::setValue(double value, bool block_signal)
{
  slider_->blockSignals(true);
  slider_->setValue(value);
  slider_->blockSignals(false);

  value_->setText(QString::number(value, 'f', decimals_) + suffix_);

  if (!block_signal)
    Q_EMIT valueChanged(value);
}

void DoubleSliderDisplay::setText(const QString& text)
{
  text_->setText(text);
}

void DoubleSliderDisplay::setMinimum(double minimum)
{
  slider_->setMinimum(minimum);
}

void DoubleSliderDisplay::setMaximum(double maximum)
{
  slider_->setMaximum(maximum);
}

void DoubleSliderDisplay::setSuffix(const QString& suffix)
{
  suffix_ = suffix;
  value_->setText(QString::number(getValue(), 'f', decimals_) + suffix_);
}

void DoubleSliderDisplay::setCenterValue(bool block_signal)
{
  const auto value = (slider_->minimum() + slider_->maximum()) / 2;
  setValue(value, block_signal);
}

void DoubleSliderDisplay::onSliderValueChanged(double value)
{
  value_->setText(QString::number(value, 'f', decimals_) + suffix_);
  Q_EMIT valueChanged(value);
}
}  // namespace qt
