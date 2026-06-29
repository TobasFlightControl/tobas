// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/progress_dialog.hpp"

#include <QApplication>
#include <QThread>

#include <tobas_math/core.hpp>

#include "tobas_qt_tools/event.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace qt
{
ProgressDialog::ProgressDialog(const QString& title, int num_steps, QWidget* parent)
  : super(parent), num_steps_(num_steps), timer_(this)
{
  assert(num_steps > 0);

  setWindowModality(Qt::WindowModal);  // ユーザーが他のUI要素と対話できないようにする
  setWindowTitle(title);
  setStep(step_);

  connect(&timer_, &QTimer::timeout, this, &self::onTimerTimeout);
}

void ProgressDialog::show()
{
  timer_.start(250ms);
  super::show();
}

void ProgressDialog::hide()
{
  timer_.stop();
  super::hide();
}

void ProgressDialog::setLabelText(const QString& text)
{
  text_ = text;
}

void ProgressDialog::setStep(int step)
{
  step_ = std::clamp(step, 0, num_steps_);
  const auto value = math::remap(step_, 0, num_steps_, minimum(), maximum());
  setValue(value);
}

void ProgressDialog::progressStep()
{
  setStep(step_ + 1);
}

void ProgressDialog::onTimerTimeout()
{
  if (text_.isEmpty()) {
    return;
  }

  // スピナーの文字を決定
  spinner_step_ = (spinner_step_ + 1) % kSpinnerFrameSize;
  const auto spinner = kSpinnerFrames[spinner_step_];

  // スピナー部分だけ等幅で表示
  super::setLabelText(
    QString(R"(<div style="text-align:center;">%1 <span style="font-family:monospace;">%2</span></div>)")
      .arg(text_)
      .arg(spinner));
}
}  // namespace qt
}  // namespace tobas
