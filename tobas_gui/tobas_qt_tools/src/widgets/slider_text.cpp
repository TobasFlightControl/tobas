#include <QHBoxLayout>
#include <QIntValidator>
#include <QDoubleValidator>

#include "tobas_qt_tools/widgets/slider_text.hpp"

#define VALUE_WIDTH 100

namespace qt
{
IntSliderTextWidget::IntSliderTextWidget(int minimum, int maximum, QWidget* parent) : super(parent)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  cols->addWidget(new QLabel(QString::number(minimum)));

  slider_ = new Slider(Qt::Horizontal);
  slider_->setRange(minimum, maximum);
  connect(slider_, &Slider::sliderReleased, this, &self::onSliderReleased);
  cols->addWidget(slider_);

  cols->addWidget(new QLabel(QString::number(maximum)));

  line_edit_ = new QLineEdit();
  line_edit_->setFixedWidth(VALUE_WIDTH);
  line_edit_->setValidator(new QIntValidator(minimum, maximum));
  connect(line_edit_, &QLineEdit::returnPressed, this, &self::onLineEditReturnPressed);
  cols->addWidget(line_edit_);
}

int IntSliderTextWidget::get() const
{
  return slider_->value();
}

void IntSliderTextWidget::set(int value)
{
  setSliderValue(value);
  setLineEditText(value);
}

void IntSliderTextWidget::onSliderReleased()
{
  const auto value = slider_->value();
  setLineEditText(value);
  Q_EMIT valueChanged(value);
}

void IntSliderTextWidget::onLineEditReturnPressed()
{
  const auto value = line_edit_->text().toInt();
  setSliderValue(value);
  Q_EMIT valueChanged(value);
}

void IntSliderTextWidget::setSliderValue(int value)
{
  slider_->blockSignals(true);
  slider_->setValue(value);
  slider_->blockSignals(false);
}

void IntSliderTextWidget::setLineEditText(int value)
{
  line_edit_->blockSignals(true);
  line_edit_->setText(QString::number(value));
  line_edit_->blockSignals(false);
}

DoubleSliderTextWidget::DoubleSliderTextWidget(double minimum, double maximum, int decimals, QWidget* parent)
  : super(parent), decimals_(decimals)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  cols->addWidget(new QLabel(QString::number(minimum, 'f', decimals)));

  slider_ = new DoubleSlider(Qt::Horizontal);
  slider_->setRange(minimum, maximum);
  connect(slider_, &Slider::sliderReleased, this, &self::onSliderReleased);
  cols->addWidget(slider_);

  cols->addWidget(new QLabel(QString::number(maximum, 'f', decimals)));

  line_edit_ = new QLineEdit();
  line_edit_->setFixedWidth(VALUE_WIDTH);
  line_edit_->setValidator(new QDoubleValidator(minimum, maximum, decimals));
  connect(line_edit_, &QLineEdit::returnPressed, this, &self::onLineEditReturnPressed);
  cols->addWidget(line_edit_);
}

double DoubleSliderTextWidget::get() const
{
  return slider_->value();
}

void DoubleSliderTextWidget::set(double value)
{
  setSliderValue(value);
  setLineEditText(value);
}

void DoubleSliderTextWidget::onSliderReleased()
{
  const auto value = slider_->value();
  setLineEditText(value);
  Q_EMIT valueChanged(value);
}

void DoubleSliderTextWidget::onLineEditReturnPressed()
{
  const auto value = line_edit_->text().toDouble();
  setSliderValue(value);
  Q_EMIT valueChanged(value);
}

void DoubleSliderTextWidget::setSliderValue(double value)
{
  slider_->blockSignals(true);
  slider_->setValue(value);
  slider_->blockSignals(false);
}

void DoubleSliderTextWidget::setLineEditText(double value)
{
  line_edit_->blockSignals(true);
  line_edit_->setText(QString::number(value, 'f', decimals_));
  line_edit_->blockSignals(false);
}
}  // namespace qt
