#include "tobas_qt_tools/widgets/labeled_double_spin_box.hpp"

#include <QHBoxLayout>
#include <QLabel>

namespace qt
{
LabeledDoubleSpinBox::LabeledDoubleSpinBox(const QString& label_text)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  const auto label = new QLabel(label_text + ":");
  cols->addWidget(label);

  data_ = new qt::DoubleSpinBox();
  data_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);  // こうしないとSpinBoxがLabelに押されてしまう
  cols->addWidget(data_);

  connect(data_, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onValueChanged);
}

double LabeledDoubleSpinBox::getValue() const
{
  return data_->value();
}

bool LabeledDoubleSpinBox::setValue(const double& value)
{
  if (value < data_->minimum() || data_->maximum() < value) {
    return false;
  }

  data_->setValue(value);
  return true;
}

void LabeledDoubleSpinBox::setDecimals(int decimals)
{
  data_->setDecimals(decimals);
}

void LabeledDoubleSpinBox::setMinimum(double minimum)
{
  data_->setMinimum(minimum);
}

void LabeledDoubleSpinBox::setMaximum(double maximum)
{
  data_->setMaximum(maximum);
}

void LabeledDoubleSpinBox::setSingleStep(double single_step)
{
  data_->setSingleStep(single_step);
}

void LabeledDoubleSpinBox::setSuffix(const QString& suffix)
{
  data_->setSuffix(suffix);
}

void LabeledDoubleSpinBox::onValueChanged(double value)
{
  Q_EMIT valueChanged(value);
}
}  // namespace qt
