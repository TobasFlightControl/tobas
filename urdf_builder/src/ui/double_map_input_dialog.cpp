#include "../../include/urdf_builder/ui/double_map_input_dialog.hpp"

namespace urdf_builder
{
namespace ui
{
DoubleMapInputDialog::DoubleMapInputDialog(QWidget* parent, const QString& title, const QStringList& field_names)
  : QDialog(parent)
{
  setWindowTitle(title);

  const auto root_layout = new QVBoxLayout(this);

  for (const auto& field_name : field_names)
  {
    const auto layout = new QHBoxLayout();
    root_layout->addLayout(layout);

    const auto label = new QLabel();
    label->setText(field_name);
    layout->addWidget(label);

    const auto spin_box = new QDoubleSpinBox();
    spin_box->setMaximum(kMaxValue);
    spin_box->setMinimum(kMinValue);
    spin_box->setValue(kDefaultValue);
    spin_box->setSingleStep(kSingleStep);
    spin_box->setDecimals(kDecimals);
    layout->addWidget(spin_box);

    field2value_[field_name] = kDefaultValue;
    spinbox2field_[spin_box] = field_name;
    connect(spin_box, SIGNAL(valueChanged(double)), this, SLOT(SpinBoxValueChanged(double)));
  }

  const auto button_box = new QDialogButtonBox();
  button_box->setObjectName(QStringLiteral("buttonBox"));
  button_box->setOrientation(Qt::Horizontal);
  button_box->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  root_layout->addWidget(button_box);

  QObject::connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
  QObject::connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));
}

const double& DoubleMapInputDialog::getValue(const QString& field) const
{
  return field2value_.at(field);
}

void DoubleMapInputDialog::SpinBoxValueChanged(double value)
{
  const auto obj = sender();
  const auto field_name = spinbox2field_[dynamic_cast<QDoubleSpinBox*>(obj)];
  field2value_[field_name] = value;
}
}  // namespace ui
}  // namespace urdf_builder
