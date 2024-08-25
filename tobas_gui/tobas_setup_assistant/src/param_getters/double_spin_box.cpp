#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/string.hpp>

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"

namespace gui
{
namespace setup_assistant
{
ParamGetterWidget_DoubleSpinBox::ParamGetterWidget_DoubleSpinBox(
  const QString& param_name,
  const QString& description_text,
  int decimals,
  double minimum,
  double maximum,
  double single_step,
  std::optional<double> _default,
  const QString& suffix)
  : super(param_name, description_text)
{
  TOBAS_CHECK(minimum <= maximum);
  TOBAS_CHECK(single_step > 0.);

  spin_box_ = new qt::DoubleSpinBox();
  rows_->addWidget(spin_box_);

  spin_box_->setDecimals(decimals);  // 桁数の設定を最初にしないと，デフォルト値などが潰れてしまう
  spin_box_->setMinimum(minimum);
  spin_box_->setMaximum(maximum);
  spin_box_->setSingleStep(single_step);
  if (_default.has_value())
  {
    TOBAS_CHECK(minimum <= _default && _default <= maximum);
    spin_box_->setValue(_default.value());
  }
  spin_box_->setSuffix(QString::fromStdString(tobas_std::convertToSuperscript(suffix.toStdString())));
  spin_box_->setFocusPolicy(Qt::StrongFocus);

  connect(spin_box_, SIGNAL(qt::DoubleSpinBox::valueChanged(double)), this, SLOT(onValueChanged(double)));
}

double ParamGetterWidget_DoubleSpinBox::get() const
{
  return spin_box_->value();
}

bool ParamGetterWidget_DoubleSpinBox::set(const double& src)
{
  if (src < spin_box_->minimum() || spin_box_->maximum() < src)
    return false;

  spin_box_->setValue(src);
  return true;
}

void ParamGetterWidget_DoubleSpinBox::onValueChanged(double value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace setup_assistant
}  // namespace gui
