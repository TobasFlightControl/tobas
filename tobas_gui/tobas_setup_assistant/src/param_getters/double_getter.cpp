#include "tobas_setup_assistant/param_getters/double_getter.hpp"

#include <QHBoxLayout>
#include <QLabel>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_string_tools/core.hpp>

namespace gui
{
namespace sa
{
DoubleGetter::DoubleGetter(const QString& name)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const auto label = new QLabel(name + ":");
  label->setFont(qt::DefaultFont(cmn::kBodyPSize));
  cols->addWidget(label);

  data_ = new qt::DoubleSpinBox();
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  cols->addWidget(data_);

  connect(data_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

double DoubleGetter::getValue() const
{
  return data_->value();
}

bool DoubleGetter::setValue(const double& value)
{
  if (value < data_->minimum() || data_->maximum() < value) {
    return false;
  }

  data_->setValue(value);
  return true;
}

void DoubleGetter::setDecimals(int decimals)
{
  data_->setDecimals(decimals);
}

void DoubleGetter::setMinimum(double minimum)
{
  data_->setMinimum(minimum);
}

void DoubleGetter::setMaximum(double maximum)
{
  data_->setMaximum(maximum);
}

void DoubleGetter::setSingleStep(double single_step)
{
  data_->setSingleStep(single_step);
}

void DoubleGetter::setSuffix(const QString& suffix)
{
  data_->setSuffix(QString::fromStdString(str::convertToSuperscript(suffix.toStdString())));
}

void DoubleGetter::onValueChanged(double value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace sa
}  // namespace gui
