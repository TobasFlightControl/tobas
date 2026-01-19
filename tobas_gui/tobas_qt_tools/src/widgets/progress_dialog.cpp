#include "tobas_qt_tools/widgets/progress_dialog.hpp"

#include <QApplication>
#include <QThread>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_qt_tools/event.hpp"

using namespace std::chrono_literals;

namespace qt
{
ProgressDialog::ProgressDialog(const QString& title, int num_steps, QWidget* parent)
  : super(parent), num_steps_(num_steps), timer_(this)
{
  TOBAS_CHECK(num_steps > 0);

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
  TOBAS_CHECK(0 <= step && step <= num_steps_);

  step_ = step;
  const auto value = math::remap(step, 0, num_steps_, minimum(), maximum());
  setValue(value);
}

void ProgressDialog::progressStep()
{
  setStep(step_ + 1);
}

void ProgressDialog::onTimerTimeout()
{
  spinner_step_ = (spinner_step_ + 1) % kSpinnerFrameSize;
  super::setLabelText(QString("%1 %2").arg(text_).arg(kSpinnerFrames[spinner_step_]));
}
}  // namespace qt
