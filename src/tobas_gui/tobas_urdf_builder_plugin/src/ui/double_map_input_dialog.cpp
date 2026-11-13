// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/ui/double_map_input_dialog.hpp"

#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>

namespace tobas::gui::ub::ui
{
DoubleMapInputDialog::DoubleMapInputDialog(QWidget* _parent, const QString& _title, const QStringList& _field_names)
  : super(_parent)
{
  setWindowTitle(_title);

  const auto root_layout = new QVBoxLayout(this);

  for (const auto& field_name : _field_names) {
    const auto layout = new QHBoxLayout();
    root_layout->addLayout(layout);

    const auto label = new QLabel(field_name);
    layout->addWidget(label);

    const auto spin_box = new QDoubleSpinBox();
    spin_box->setMaximum(1000.0);
    spin_box->setMinimum(0.0);
    spin_box->setValue(0.0);
    spin_box->setSingleStep(0.1);
    spin_box->setDecimals(6);
    layout->addWidget(spin_box);

    field2value_[field_name] = spin_box->value();
    spinbox2field_[spin_box] = field_name;
    connect(spin_box, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &self::onSpinBoxValueChanged);
  }

  const auto button_box = new QDialogButtonBox();
  button_box->setObjectName(QStringLiteral("buttonBox"));
  button_box->setOrientation(Qt::Horizontal);
  button_box->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  root_layout->addWidget(button_box);

  connect(button_box, &QDialogButtonBox::accepted, this, &self::accept);
  connect(button_box, &QDialogButtonBox::rejected, this, &self::reject);
}

const double& DoubleMapInputDialog::getValue(const QString& _field) const
{
  return field2value_.at(_field);
}

void DoubleMapInputDialog::onSpinBoxValueChanged(double _value)
{
  const auto obj = qt::qPointerCast<QDoubleSpinBox>(sender());
  const auto field_name = spinbox2field_.at(obj);
  field2value_[field_name] = _value;
}
}  // namespace tobas::gui::ub::ui
