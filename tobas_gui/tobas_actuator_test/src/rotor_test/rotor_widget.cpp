// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/rotor_test/rotor_widget.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_qt_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace at
{
RotorWidget::RotorWidget()
{
  text_ = new QLabel();
  text_->setAlignment(Qt::AlignCenter);

  cur_rpm_meter_ = new QwtThermo();
  cur_rpm_meter_->setLowerBound(0);
  cur_rpm_meter_->setPipeWidth(50);

  tar_rpm_slider_ = new QwtSlider(Qt::Vertical);
  tar_rpm_slider_->setLowerBound(0);
  tar_rpm_slider_->setScalePosition(QwtSlider::NoScale);
  tar_rpm_slider_->setTrough(false);
  tar_rpm_slider_->setGroove(true);

  constexpr int kMinRotorCtrlGain = 0;
  constexpr int kMaxRotorCtrlGain = 30;
  gain_slider_ = new QwtSlider(Qt::Vertical);
  gain_slider_->setScale(kMinRotorCtrlGain, kMaxRotorCtrlGain);
  gain_slider_->setTotalSteps(kMaxRotorCtrlGain - kMinRotorCtrlGain);  // Step size is 1.
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
  const QSignalBlocker block(this);

  cur_rpm_meter_->setValue(0);
  tar_rpm_slider_->setValue(0);
  gain_slider_->setValue(0);

  setCurrentRPMBox(0);
  setTargetRPMBox(0);
  setGainBox(0);
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

void RotorWidget::setText(const QString& _text)
{
  text_->setText(_text);
}

void RotorWidget::setMaximumRPM(int _rpm)
{
  // Prevent `valueChanged` from being emitted while setting the slider range.
  const QSignalBlocker cur_rpm_block(cur_rpm_meter_);
  const QSignalBlocker tar_rpm_block(tar_rpm_slider_);

  cur_rpm_meter_->setUpperBound(_rpm);

  tar_rpm_slider_->setUpperBound(_rpm);
  tar_rpm_slider_->setTotalSteps(_rpm);  // Step size is 1.
}

void RotorWidget::setCurrentRPM(int _rpm)
{
  cur_rpm_meter_->setValue(_rpm);
  setCurrentRPMBox(_rpm);
}

void RotorWidget::setTargetRPM(int _rpm)
{
  tar_rpm_slider_->setValue(_rpm);
  setTargetRPMBox(_rpm);
}

void RotorWidget::setGain(int _gain)
{
  gain_slider_->setValue(_gain);
  setGainBox(_gain);
}

void RotorWidget::setCurrentRPMBox(int _rpm)
{
  cur_rpm_box_->setText(rpmToText(_rpm));
}

void RotorWidget::setTargetRPMBox(int _rpm)
{
  tar_rpm_box_->setText(rpmToText(_rpm));
}

void RotorWidget::setGainBox(int _gain)
{
  gain_box_->setText(QString::number(_gain));
}

QString RotorWidget::rpmToText(int _rpm)
{
  return QString::number(_rpm) + " RPM";
}

void RotorWidget::onTargetRPMChanged(int _rpm)
{
  setTargetRPMBox(_rpm);
  Q_EMIT targetRPMChanged(_rpm);
}

void RotorWidget::onGainChanged(int _gain)
{
  setGainBox(_gain);
  Q_EMIT gainChanged(_gain);
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
