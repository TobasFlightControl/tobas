#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_hardware_setup/rotor_test/rotor_widget.hpp"

namespace gui
{
namespace hardware_setup
{
RotorWidget::RotorWidget()
{
  text_ = new QLabel();
  text_->setAlignment(Qt::AlignCenter);

  cur_rpm_bar_ = new SpeedmeterWidget();
  cur_rpm_bar_->setMinimumValue(0);

  tar_rpm_slider_ = new qt::Slider(Qt::Vertical);
  tar_rpm_slider_->setMinimum(0);

  gain_slider_ = new qt::Slider(Qt::Vertical);
  gain_slider_->setMinimum(tobas::kMinRotorCtrlGain);
  gain_slider_->setMaximum(tobas::kMaxRotorCtrlGain);

  cur_rpm_box_ = new QLineEdit();
  cur_rpm_box_->setAlignment(Qt::AlignRight);
  cur_rpm_box_->setReadOnly(true);
  cur_rpm_box_->setFocusPolicy(Qt::NoFocus);

  tar_rpm_box_ = new QLineEdit();
  tar_rpm_box_->setAlignment(Qt::AlignRight);
  tar_rpm_box_->setReadOnly(true);
  tar_rpm_box_->setFocusPolicy(Qt::NoFocus);

  gain_box_ = new QLineEdit();
  gain_box_->setAlignment(Qt::AlignRight);
  gain_box_->setReadOnly(true);
  gain_box_->setFocusPolicy(Qt::NoFocus);

  // Layout
  const auto rpm_cols = new QHBoxLayout();
  rpm_cols->addWidget(cur_rpm_bar_, 2);
  rpm_cols->addStretch(1);
  rpm_cols->addWidget(tar_rpm_slider_, 1);
  rpm_cols->addStretch(1);

  const auto rpm_form = new QFormLayout();
  rpm_form->addRow("Current", cur_rpm_box_);
  rpm_form->addRow("Target", tar_rpm_box_);

  const auto gain_form = new QFormLayout();
  gain_form->addRow("Gain", gain_box_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(text_, 0);
  rows->addLayout(rpm_cols, 1);
  rows->addLayout(rpm_form, 0);
  qt::addWidgetCenter(gain_slider_, rows, 1);
  rows->addLayout(gain_form, 0);

  setLayout(rows);

  // Connection
  connect(tar_rpm_slider_, &qt::Slider::valueChanged, this, &self::onTargetRPMChanged);
  connect(gain_slider_, &qt::Slider::valueChanged, this, &self::onGainChanged);
}

QString RotorWidget::getText() const
{
  return text_->text();
}

int RotorWidget::getCurrentRPM() const
{
  return cur_rpm_bar_->getValue();
}

int RotorWidget::getTargetRPM() const
{
  return tar_rpm_slider_->value();
}

int RotorWidget::getGain() const
{
  return gain_slider_->value();
}

void RotorWidget::setText(const QString& text)
{
  text_->setText(text);
}

void RotorWidget::setMaximumRPM(int rpm)
{
  cur_rpm_bar_->setMaximumValue(rpm);
  tar_rpm_slider_->setMaximum(rpm);
}

void RotorWidget::setCurrentRPM(int rpm)
{
  cur_rpm_bar_->setValue(rpm);
  cur_rpm_box_->setText(QString::number(rpm) + " RPM");
}

void RotorWidget::setTargetRPM(int rpm)
{
  tar_rpm_slider_->setValue(rpm);
  tar_rpm_box_->setText(QString::number(rpm) + " RPM");
}

void RotorWidget::setGain(int gain)
{
  gain_slider_->setValue(gain);
  gain_box_->setText(QString::number(gain));
}

void RotorWidget::onTargetRPMChanged(int rpm)
{
  Q_EMIT targetRPMChanged(rpm);
}

void RotorWidget::onGainChanged(int gain)
{
  Q_EMIT gainChanged(gain);
}
}  // namespace hardware_setup
}  // namespace gui
