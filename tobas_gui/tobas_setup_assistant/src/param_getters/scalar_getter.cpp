#include <QLabel>
#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/string.hpp>
#include <tobas_qt_tools/font.hpp>

#include "tobas_setup_assistant/param_getters/scalar_getter.hpp"
#include "tobas_setup_assistant/common.hpp"

namespace gui
{
namespace setup_assistant
{
IntGetter::IntGetter(
  const QString& name,
  int minimum,
  int maximum,
  int single_step,
  std::optional<int> _default,
  const QString& suffix)
{
  auto cols = new QHBoxLayout();
  setLayout(cols);

  auto label = new QLabel(name + ":");
  label->setFont(qt::DefaultFont(kBodyPSize));
  cols->addWidget(label);

  data_ = new qt::SpinBox();
  data_->setMinimum(minimum);
  data_->setMaximum(maximum);
  data_->setSingleStep(single_step);
  if (_default.has_value())
  {
    TOBAS_CHECK(minimum <= _default && _default <= maximum);
    data_->setValue(_default.value());
  }
  data_->setSuffix(QString::fromStdString(tobas_std::convertToSuperscript(suffix.toStdString())));
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  data_->setFocusPolicy(Qt::StrongFocus);
  cols->addWidget(data_);

  connect(data_, SIGNAL(qt::SpinBox::valueChanged(int)), this, SLOT(onValueChanged(int)));
}

int IntGetter::get() const
{
  return data_->value();
}

bool IntGetter::set(const int& value)
{
  if (value < data_->minimum() || data_->maximum() < value)
    return false;

  data_->setValue(value);
  return true;
}

void IntGetter::onValueChanged(int value)
{
  Q_EMIT valueChanged(value);
}

DoubleGetter::DoubleGetter(
  const QString& name,
  int decimals,
  double minimum,
  double maximum,
  double single_step,
  std::optional<double> _default,
  const QString& suffix)
{
  auto cols = new QHBoxLayout();
  setLayout(cols);

  auto label = new QLabel(name + ":");
  label->setFont(qt::DefaultFont(kBodyPSize));
  cols->addWidget(label);

  data_ = new qt::DoubleSpinBox();
  data_->setDecimals(decimals);
  data_->setMinimum(minimum);
  data_->setMaximum(maximum);
  data_->setSingleStep(single_step);
  if (_default.has_value())
  {
    TOBAS_CHECK(minimum <= _default && _default <= maximum);
    data_->setValue(_default.value());
  }
  data_->setSuffix(QString::fromStdString(tobas_std::convertToSuperscript(suffix.toStdString())));
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  data_->setFocusPolicy(Qt::StrongFocus);
  cols->addWidget(data_);

  connect(data_, SIGNAL(qt::DoubleSpinBox::valueChanged(double)), this, SLOT(onValueChanged(double)));
}

double DoubleGetter::get() const
{
  return data_->value();
}

bool DoubleGetter::set(const double& value)
{
  if (value < data_->minimum() || data_->maximum() < value)
    return false;

  data_->setValue(value);
  return true;
}

void DoubleGetter::onValueChanged(double value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace setup_assistant
}  // namespace gui
