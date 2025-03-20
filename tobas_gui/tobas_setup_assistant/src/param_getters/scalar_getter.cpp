#include <QLabel>
#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/param_getters/scalar_getter.hpp"
#include "tobas_setup_assistant/constants.hpp"

namespace gui
{
namespace sa
{
IntGetter::IntGetter(const QString& name)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const auto label = new QLabel(name + ":");
  label->setFont(qt::DefaultFont(kBodyPSize));
  cols->addWidget(label);

  data_ = new qt::SpinBox();
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  data_->setFocusPolicy(Qt::StrongFocus);
  cols->addWidget(data_);

  connect(data_, QOverload<int>::of(&qt::SpinBox::valueChanged), this, &self::onValueChanged);
}

int IntGetter::getValue() const
{
  return data_->value();
}

bool IntGetter::setValue(const int& value)
{
  if (value < data_->minimum() || data_->maximum() < value)
    return false;

  data_->setValue(value);
  return true;
}

void IntGetter::setMinimum(int minimum)
{
  data_->setMinimum(minimum);
}

void IntGetter::setMaximum(int maximum)
{
  data_->setMaximum(maximum);
}

void IntGetter::setSingleStep(int single_step)
{
  data_->setSingleStep(single_step);
}

void IntGetter::setSuffix(const QString& suffix)
{
  data_->setSuffix(QString::fromStdString(str::convertToSuperscript(suffix.toStdString())));
}

void IntGetter::onValueChanged(int value)
{
  Q_EMIT valueChanged(value);
}

DoubleGetter::DoubleGetter(const QString& name)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const auto label = new QLabel(name + ":");
  label->setFont(qt::DefaultFont(kBodyPSize));
  cols->addWidget(label);

  data_ = new qt::DoubleSpinBox();
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  data_->setFocusPolicy(Qt::StrongFocus);
  cols->addWidget(data_);

  connect(data_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

double DoubleGetter::getValue() const
{
  return data_->value();
}

bool DoubleGetter::setValue(const double& value)
{
  if (value < data_->minimum() || data_->maximum() < value)
    return false;

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
