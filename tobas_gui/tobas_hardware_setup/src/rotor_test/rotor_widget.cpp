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

  cur_rpm_meter_ = new QwtThermo();
  cur_rpm_meter_->setLowerBound(0);
  cur_rpm_meter_->setPipeWidth(kPipeWidth);

  tar_rpm_slider_ = new QwtSlider(Qt::Vertical);
  tar_rpm_slider_->setLowerBound(0);
  tar_rpm_slider_->setScalePosition(QwtSlider::NoScale);
  tar_rpm_slider_->setTrough(false);
  tar_rpm_slider_->setGroove(true);

  gain_slider_ = new QwtSlider(Qt::Vertical);
  gain_slider_->setScale(tobas::kMinRotorCtrlGain, tobas::kMaxRotorCtrlGain);
  gain_slider_->setTotalSteps(tobas::kMaxRotorCtrlGain - tobas::kMinRotorCtrlGain);  // 1刻み
  gain_slider_->setScalePosition(QwtSlider::TrailingScale);
  gain_slider_->setTrough(false);
  gain_slider_->setGroove(true);

  cur_rpm_box_ = new qt::FramedLabel();
  tar_rpm_box_ = new qt::FramedLabel();
  gain_box_ = new qt::FramedLabel();

  // Layout
  const auto rpm_cols = new QHBoxLayout();
  rpm_cols->addWidget(cur_rpm_meter_, 3);
  rpm_cols->addWidget(tar_rpm_slider_, 1);

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
  connect(tar_rpm_slider_, &QwtSlider::valueChanged, this, &self::onTargetRPMChanged);
  connect(gain_slider_, &QwtSlider::valueChanged, this, &self::onGainChanged);

  reset();
}

void RotorWidget::reset()
{
  blockSignals(true);

  text_->clear();

  cur_rpm_meter_->setValue(0);
  tar_rpm_slider_->setValue(0);
  gain_slider_->setValue(0);

  setCurrentRPMBox(0);
  setTargetRPMBox(0);
  setGainBox(0);

  blockSignals(false);
}

QString RotorWidget::getText() const
{
  return text_->text();
}

int RotorWidget::getCurrentRPM() const
{
  return cur_rpm_meter_->value();
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
  cur_rpm_meter_->setUpperBound(rpm);

  tar_rpm_slider_->setUpperBound(rpm);
  tar_rpm_slider_->setTotalSteps(rpm);  // 1刻み
}

void RotorWidget::setCurrentRPM(int rpm)
{
  cur_rpm_meter_->setValue(rpm);
  setCurrentRPMBox(rpm);
}

void RotorWidget::setTargetRPM(int rpm)
{
  tar_rpm_slider_->setValue(rpm);
  setTargetRPMBox(rpm);
}

void RotorWidget::setGain(int gain)
{
  gain_slider_->setValue(gain);
  setGainBox(gain);
}

void RotorWidget::setCurrentRPMBox(int rpm)
{
  cur_rpm_box_->setText(rpmToText(rpm));
}

void RotorWidget::setTargetRPMBox(int rpm)
{
  tar_rpm_box_->setText(rpmToText(rpm));
}

void RotorWidget::setGainBox(int gain)
{
  gain_box_->setText(QString::number(gain));
}

QString RotorWidget::rpmToText(int rpm)
{
  return QString::number(rpm) + " RPM";
}

void RotorWidget::onTargetRPMChanged(int rpm)
{
  setTargetRPMBox(rpm);
  Q_EMIT targetRPMChanged(rpm);
}

void RotorWidget::onGainChanged(int gain)
{
  setGainBox(gain);
  Q_EMIT gainChanged(gain);
}
}  // namespace hardware_setup
}  // namespace gui
