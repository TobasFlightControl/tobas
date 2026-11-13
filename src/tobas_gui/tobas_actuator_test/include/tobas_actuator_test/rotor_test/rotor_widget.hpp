// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <qwt/qwt_slider.h>
#include <qwt/qwt_thermo.h>
#include <QLabel>

#include <tobas_qt_tools/widgets/framed_label.hpp>

namespace tobas::gui::at
{
class RotorWidget : public QWidget
{
  Q_OBJECT

  using self = RotorWidget;

Q_SIGNALS:
  void targetRPMChanged(int _rpm);
  void gainChanged(int _gain);

public:
  explicit RotorWidget();

  void reset();

  QString getText() const;
  int getCurrentRPM() const;
  int getTargetRPM() const;
  int getGain() const;

  void setText(const QString& _text);
  void setMaximumRPM(int _rpm);
  void setCurrentRPM(int _rpm);
  void setTargetRPM(int _rpm);
  void setGain(int _gain);

private:
  QLabel* text_;
  QwtThermo* cur_rpm_meter_;
  QwtSlider* tar_rpm_slider_;
  QwtSlider* gain_slider_;
  qt::FramedLabel* cur_rpm_box_;
  qt::FramedLabel* tar_rpm_box_;
  qt::FramedLabel* gain_box_;

  void setCurrentRPMBox(int _rpm);
  void setTargetRPMBox(int _rpm);
  void setGainBox(int _gain);

  static QString rpmToText(int _rpm);

private Q_SLOTS:
  void onTargetRPMChanged(int _rpm);
  void onGainChanged(int _gain);
};
}  // namespace tobas::gui::at
