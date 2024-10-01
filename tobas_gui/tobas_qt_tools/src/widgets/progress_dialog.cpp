#include <QCoreApplication>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_qt_tools/widgets/progress_dialog.hpp"

namespace qt
{
ProgressDialog::ProgressDialog(const QString& title, int num_steps, QWidget* parent)
  : super(parent), num_steps_(num_steps)
{
  TOBAS_CHECK(num_steps > 0);

  setWindowModality(Qt::WindowModal);  // ユーザーが他のUI要素と対話できないようにする
  setWindowTitle(title);
  setStep(step_);
}

void ProgressDialog::show()
{
  super::show();
  reflesh();
}

void ProgressDialog::setValue(int value)
{
  super::setValue(value);
  reflesh();
}

void ProgressDialog::setLabelText(const QString& text)
{
  super::setLabelText(text);
  reflesh();
}

void ProgressDialog::setStep(int step)
{
  TOBAS_CHECK(0 <= step && step <= num_steps_);

  step_ = step;
  const auto value = math::remap(step, 0, num_steps_, minimum(), maximum());
  setValue(value);
  reflesh();
}

void ProgressDialog::progressStep()
{
  setStep(step_ + 1);
}

void ProgressDialog::reflesh()
{
  // FIXME: processEvents()を複数回実行してもProgressが更新されない．
  QCoreApplication::processEvents();
}
}  // namespace qt
