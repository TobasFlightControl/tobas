#include <QVBoxLayout>
#include <QHBoxLayout>

#include "tobas_qt_tools/widgets/slider_display.hpp"

#define TEXT_PSIZE 9

namespace qt
{
IntSliderDisplay::IntSliderDisplay(QWidget* parent) : super(parent)
{
  QFont font("Default", TEXT_PSIZE, QFont::Bold);

  auto rows = new QVBoxLayout();
  setLayout(rows);

  auto cols = new QHBoxLayout();
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

  update();
  connect(slider_, SIGNAL(Slider::valueChanged(int)), this, SLOT(onValueChanged(int)));
}

void IntSliderDisplay::update()
{
  const auto value = getValue();
  value_->setText(QString::number(value) + suffix_);
  Q_EMIT valueChanged(value);
}

int IntSliderDisplay::getValue() const
{
  return slider_->value();
}

void IntSliderDisplay::setValue(int value)
{
  slider_->setValue(value);
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
  update();
}

void IntSliderDisplay::setCenterValue()
{
  const auto value = (slider_->minimum() + slider_->maximum()) / 2;
  setValue(value);
}

void IntSliderDisplay::onValueChanged(int)
{
  update();
}

DoubleSliderDisplay::DoubleSliderDisplay(int decimals, QWidget* parent) : super(parent), decimals_(decimals)
{
  QFont font("Default", TEXT_PSIZE, QFont::Bold);

  auto rows = new QVBoxLayout();
  setLayout(rows);

  auto cols = new QHBoxLayout();
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

  update();
  connect(slider_, SIGNAL(Slider::valueChanged(double)), this, SLOT(onValueChanged(double)));
}

void DoubleSliderDisplay::update()
{
  const auto value = getValue();
  value_->setText(QString::number(value, 'f', decimals_) + suffix_);
  Q_EMIT valueChanged(value);
}

double DoubleSliderDisplay::getValue() const
{
  return slider_->value();
}

void DoubleSliderDisplay::setValue(double value)
{
  slider_->setValue(value);
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
  update();
}

void DoubleSliderDisplay::setCenterValue()
{
  const auto value = (slider_->minimum() + slider_->maximum()) / 2;
  setValue(value);
}

void DoubleSliderDisplay::onValueChanged(double)
{
  update();
}
}  // namespace qt
