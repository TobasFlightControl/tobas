// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/fixed_wing/aero_coefs.hpp"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/format.hpp>

#include "tobas_setup_assistant/setting_tabs/fixed_wing/constants.hpp"
#include "tobas_setup_assistant/setting_tabs/fixed_wing/vspaero.hpp"

namespace tobas::gui::sa::fw
{
AerodynamicsCoefficientsWidget::AerodynamicsCoefficientsWidget()
{
  setFrameShape(QFrame::NoFrame);

  const auto load_button = new QPushButton("Load VSPAERO Output");
  load_button->setFixedWidth(200);
  load_button->setToolTip(
    "Import aerodynamic coefficients from a VSPAERO stability output file (.stab), replacing the current values.");

  c_lift_0_ = new qt::DoubleSpinBox();
  c_lift_0_->setToolTip("Lift coefficient at zero angle of attack. Dimensionless intercept of the lift model.");
  c_lift_0_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_lift_0_->setDecimals(kStabilityCoefDecimals);
  c_lift_0_->setSuffix(" [-]");

  c_lift_alpha_ = new qt::DoubleSpinBox();
  c_lift_alpha_->setToolTip("Change in lift coefficient per radian of angle of attack.");
  c_lift_alpha_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_lift_alpha_->setDecimals(kStabilityCoefDecimals);
  c_lift_alpha_->setSuffix(" [/rad]");

  c_drag_0_ = new qt::DoubleSpinBox();
  c_drag_0_->setToolTip("Drag coefficient at zero angle of attack. Dimensionless intercept of the drag model.");
  c_drag_0_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_drag_0_->setDecimals(kStabilityCoefDecimals);
  c_drag_0_->setSuffix(" [-]");

  c_drag_alpha_ = new qt::DoubleSpinBox();
  c_drag_alpha_->setToolTip("Change in drag coefficient per radian of angle of attack in the linear drag model.");
  c_drag_alpha_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_drag_alpha_->setDecimals(kStabilityCoefDecimals);
  c_drag_alpha_->setSuffix(" [/rad]");

  c_drag_alpha2_ = new qt::DoubleSpinBox();
  c_drag_alpha2_->setToolTip(
    "Second derivative of drag coefficient with respect to angle of attack in the linear drag model.");
  c_drag_alpha2_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_drag_alpha2_->setDecimals(kStabilityCoefDecimals);
  c_drag_alpha2_->setSuffix(" [/rad^2]");

  c_side_beta_ = new qt::DoubleSpinBox();
  c_side_beta_->setToolTip("Change in side-force coefficient per radian of sideslip angle.");
  c_side_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_side_beta_->setDecimals(kStabilityCoefDecimals);
  c_side_beta_->setSuffix(" [/rad]");

  c_side_p_ = new qt::DoubleSpinBox();
  c_side_p_->setToolTip("Change in side-force coefficient with respect to roll rate p.");
  c_side_p_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_side_p_->setDecimals(kStabilityCoefDecimals);
  c_side_p_->setSuffix(" [/rad]");

  c_side_r_ = new qt::DoubleSpinBox();
  c_side_r_->setToolTip("Change in side-force coefficient with respect to yaw rate r.");
  c_side_r_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_side_r_->setDecimals(kStabilityCoefDecimals);
  c_side_r_->setSuffix(" [/rad]");

  c_roll_beta_ = new qt::DoubleSpinBox();
  c_roll_beta_->setToolTip("Change in rolling-moment coefficient per radian of sideslip angle.");
  c_roll_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_roll_beta_->setDecimals(kStabilityCoefDecimals);
  c_roll_beta_->setSuffix(" [/rad]");

  c_roll_p_ = new qt::DoubleSpinBox();
  c_roll_p_->setToolTip("Rolling-moment response to roll rate p (roll damping derivative).");
  c_roll_p_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_roll_p_->setDecimals(kStabilityCoefDecimals);
  c_roll_p_->setSuffix(" [s/rad]");

  c_roll_r_ = new qt::DoubleSpinBox();
  c_roll_r_->setToolTip("Rolling-moment response to yaw rate r (yaw-to-roll coupling derivative).");
  c_roll_r_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_roll_r_->setDecimals(kStabilityCoefDecimals);
  c_roll_r_->setSuffix(" [s/rad]");

  c_pitch_0_ = new qt::DoubleSpinBox();
  c_pitch_0_->setToolTip(
    "Pitching-moment coefficient at zero angle of attack, sideslip, and angular rates, "
    "referenced to the aerodynamic center.");
  c_pitch_0_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_0_->setDecimals(kStabilityCoefDecimals);
  c_pitch_0_->setSuffix(" [-]");

  c_pitch_alpha_ = new qt::DoubleSpinBox();
  c_pitch_alpha_->setToolTip(
    "Change in pitching-moment coefficient per radian of angle of attack, referenced to the aerodynamic center.");
  c_pitch_alpha_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_alpha_->setDecimals(kStabilityCoefDecimals);
  c_pitch_alpha_->setSuffix(" [/rad]");

  c_pitch_abs_beta_ = new qt::DoubleSpinBox();
  c_pitch_abs_beta_->setToolTip(
    "Change in pitching-moment coefficient per radian of the absolute sideslip angle, "
    "referenced to the aerodynamic center.");
  c_pitch_abs_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_abs_beta_->setDecimals(kStabilityCoefDecimals);
  c_pitch_abs_beta_->setSuffix(" [/rad]");

  c_pitch_alpha_rate_ = new qt::DoubleSpinBox();
  c_pitch_alpha_rate_->setToolTip(
    "Pitching-moment response to the rate of change of angle of attack. "
    "Importing VSPAERO output resets this value to zero; enter a separate estimate if needed.");
  c_pitch_alpha_rate_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_alpha_rate_->setDecimals(kStabilityCoefDecimals);
  c_pitch_alpha_rate_->setSuffix(" [s/rad]");

  c_pitch_q_ = new qt::DoubleSpinBox();
  c_pitch_q_->setToolTip("Pitching-moment response to pitch rate q (pitch damping derivative).");
  c_pitch_q_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_q_->setDecimals(kStabilityCoefDecimals);
  c_pitch_q_->setSuffix(" [s/rad]");

  c_yaw_beta_ = new qt::DoubleSpinBox();
  c_yaw_beta_->setToolTip(
    "Change in yawing-moment coefficient per radian of sideslip angle, referenced to the aerodynamic center.");
  c_yaw_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_yaw_beta_->setDecimals(kStabilityCoefDecimals);
  c_yaw_beta_->setSuffix(" [/rad]");

  c_yaw_p_ = new qt::DoubleSpinBox();
  c_yaw_p_->setToolTip("Yawing-moment response to roll rate p (roll-to-yaw coupling derivative).");
  c_yaw_p_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_yaw_p_->setDecimals(kStabilityCoefDecimals);
  c_yaw_p_->setSuffix(" [s/rad]");

  c_yaw_r_ = new qt::DoubleSpinBox();
  c_yaw_r_->setToolTip("Yawing-moment response to yaw rate r (yaw damping derivative).");
  c_yaw_r_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_yaw_r_->setDecimals(kStabilityCoefDecimals);
  c_yaw_r_->setSuffix(" [s/rad]");

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(load_button);
  button_cols->addStretch(1);

  form_ = new qt::FormLayout();
  form_->addRow("c_lift_0", c_lift_0_);
  form_->labelForField(c_lift_0_)->setToolTip(c_lift_0_->toolTip());
  form_->addRow("c_lift_alpha", c_lift_alpha_);
  form_->labelForField(c_lift_alpha_)->setToolTip(c_lift_alpha_->toolTip());
  form_->addRow("c_drag_0", c_drag_0_);
  form_->labelForField(c_drag_0_)->setToolTip(c_drag_0_->toolTip());
  form_->addRow("c_drag_alpha", c_drag_alpha_);
  form_->labelForField(c_drag_alpha_)->setToolTip(c_drag_alpha_->toolTip());
  form_->addRow("c_drag_alpha2", c_drag_alpha2_);
  form_->labelForField(c_drag_alpha_)->setToolTip(c_drag_alpha2_->toolTip());
  form_->addRow("c_side_beta", c_side_beta_);
  form_->labelForField(c_side_beta_)->setToolTip(c_side_beta_->toolTip());
  form_->addRow("c_side_p", c_side_p_);
  form_->labelForField(c_side_p_)->setToolTip(c_side_p_->toolTip());
  form_->addRow("c_side_r", c_side_r_);
  form_->labelForField(c_side_r_)->setToolTip(c_side_r_->toolTip());
  form_->addRow("c_roll_beta", c_roll_beta_);
  form_->labelForField(c_roll_beta_)->setToolTip(c_roll_beta_->toolTip());
  form_->addRow("c_roll_p", c_roll_p_);
  form_->labelForField(c_roll_p_)->setToolTip(c_roll_p_->toolTip());
  form_->addRow("c_roll_r", c_roll_r_);
  form_->labelForField(c_roll_r_)->setToolTip(c_roll_r_->toolTip());
  form_->addRow("c_pitch_0", c_pitch_0_);
  form_->labelForField(c_pitch_0_)->setToolTip(c_pitch_0_->toolTip());
  form_->addRow("c_pitch_alpha", c_pitch_alpha_);
  form_->labelForField(c_pitch_alpha_)->setToolTip(c_pitch_alpha_->toolTip());
  form_->addRow("c_pitch_abs_beta", c_pitch_abs_beta_);
  form_->labelForField(c_pitch_abs_beta_)->setToolTip(c_pitch_abs_beta_->toolTip());
  form_->addRow("c_pitch_alpha_rate", c_pitch_alpha_rate_);
  form_->labelForField(c_pitch_alpha_rate_)->setToolTip(c_pitch_alpha_rate_->toolTip());
  form_->addRow("c_pitch_q", c_pitch_q_);
  form_->labelForField(c_pitch_q_)->setToolTip(c_pitch_q_->toolTip());
  form_->addRow("c_yaw_beta", c_yaw_beta_);
  form_->labelForField(c_yaw_beta_)->setToolTip(c_yaw_beta_->toolTip());
  form_->addRow("c_yaw_p", c_yaw_p_);
  form_->labelForField(c_yaw_p_)->setToolTip(c_yaw_p_->toolTip());
  form_->addRow("c_yaw_r", c_yaw_r_);
  form_->labelForField(c_yaw_r_)->setToolTip(c_yaw_r_->toolTip());

  const auto root_rows = new QVBoxLayout();
  setLayout(root_rows);  // Do not pass `this` to the layout constructor.
  root_rows->addLayout(button_cols, 0);
  root_rows->addSpacing(16);
  root_rows->addLayout(form_, 1);

  // Connection
  connect(load_button, &QPushButton::clicked, this, &self::onLoadButtonClicked);
}

void AerodynamicsCoefficientsWidget::updateInternalDataStructures()
{
}

void AerodynamicsCoefficientsWidget::setToDefaults()
{
  c_lift_0_->setValue(0.2127);
  c_lift_alpha_->setValue(10.806);
  c_drag_0_->setValue(0.136);
  c_drag_alpha_->setValue(0.6737);
  c_drag_alpha2_->setValue(1.6);
  c_side_beta_->setValue(-0.3073);
  c_side_p_->setValue(0.0);
  c_side_r_->setValue(0.0);
  c_roll_beta_->setValue(-0.0154);
  c_roll_p_->setValue(-0.1647);
  c_roll_r_->setValue(0.0117);
  c_pitch_0_->setValue(0.0435);
  c_pitch_alpha_->setValue(-2.969);
  c_pitch_abs_beta_->setValue(0.0);
  c_pitch_alpha_rate_->setValue(0.0);
  c_pitch_q_->setValue(-106.1542);
  c_yaw_beta_->setValue(0.043);
  c_yaw_p_->setValue(0.0);
  c_yaw_r_->setValue(-0.0827);
}

bool AerodynamicsCoefficientsWidget::isValid()
{
  return true;
}

YAML::Node AerodynamicsCoefficientsWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (int row = 0; row < form_->rowCount(); ++row) {
    const auto label = qt::qConstPointerCast<QLabel>(form_->getLabel(row));
    const auto widget = qt::qConstPointerCast<qt::DoubleSpinBox>(form_->getWidget(row));
    node[label->text().toStdString()] = yaml::format(widget->value());
  }

  return node;
}

void AerodynamicsCoefficientsWidget::load(const YAML::Node& _node)
{
  for (int row = 0; row < form_->rowCount(); ++row) {
    const auto label = qt::qConstPointerCast<QLabel>(form_->getLabel(row));
    const auto widget = qt::qPointerCast<qt::DoubleSpinBox>(form_->getWidget(row));
    widget->setValue(_node[label->text().toStdString()].as<double>());
  }
}

double AerodynamicsCoefficientsWidget::c_lift_0() const
{
  return c_lift_0_->value();
}

double AerodynamicsCoefficientsWidget::c_lift_alpha() const
{
  return c_lift_alpha_->value();
}

double AerodynamicsCoefficientsWidget::c_drag_0() const
{
  return c_drag_0_->value();
}

double AerodynamicsCoefficientsWidget::c_drag_alpha() const
{
  return c_drag_alpha_->value();
}

double AerodynamicsCoefficientsWidget::c_drag_alpha2() const
{
  return c_drag_alpha2_->value();
}

double AerodynamicsCoefficientsWidget::c_side_beta() const
{
  return c_side_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_side_p() const
{
  return c_side_p_->value();
}

double AerodynamicsCoefficientsWidget::c_side_r() const
{
  return c_side_r_->value();
}

double AerodynamicsCoefficientsWidget::c_roll_beta() const
{
  return c_roll_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_roll_p() const
{
  return c_roll_p_->value();
}

double AerodynamicsCoefficientsWidget::c_roll_r() const
{
  return c_roll_r_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_0() const
{
  return c_pitch_0_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_alpha() const
{
  return c_pitch_alpha_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_abs_beta() const
{
  return c_pitch_abs_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_alpha_rate() const
{
  return c_pitch_alpha_rate_->value();
}

double AerodynamicsCoefficientsWidget::c_pitch_q() const
{
  return c_pitch_q_->value();
}

double AerodynamicsCoefficientsWidget::c_yaw_beta() const
{
  return c_yaw_beta_->value();
}

double AerodynamicsCoefficientsWidget::c_yaw_p() const
{
  return c_yaw_p_->value();
}

double AerodynamicsCoefficientsWidget::c_yaw_r() const
{
  return c_yaw_r_->value();
}

void AerodynamicsCoefficientsWidget::onLoadButtonClicked()
{
  // Get the previously opened path.
  constexpr char kLastOpenedDirKey[] = "fixed_wing/aero_coefs/last_opened_dir";
  const auto last_opened_dir = settings_store_.value(kLastOpenedDirKey, QDir::homePath()).toString();

  // Get the params path.
  const auto file_path = QFileDialog::getOpenFileName(
    this,
    "Select OpenVSP Stability Output File",
    last_opened_dir,
    "OpenVSP Stability Derivatives (*.stab)",
    nullptr,
    QFileDialog::DontUseNativeDialog);

  // Return without doing anything if canceled.
  if (file_path.isEmpty()) {
    return;
  }

  // Save the directory opened by the user.
  const auto par_dir = QFileInfo(file_path).absolutePath();
  settings_store_.setValue(kLastOpenedDirKey, par_dir);

  // Load parameters.
  const auto coefs = parseVspaero(file_path.toStdString());
  if (!coefs) {
    qt::qErrorBox(this, "Failed to read coefficients:\n\n" + QString::fromStdString(coefs.error()));
    return;
  }

  // Apply loaded parameters to the form.
  c_lift_0_->setValue(coefs->c_lift_0);
  c_lift_alpha_->setValue(coefs->c_lift_alpha);
  c_drag_0_->setValue(coefs->c_drag_0);
  c_drag_alpha_->setValue(coefs->c_drag_alpha);
  c_drag_alpha2_->setValue(coefs->c_drag_alpha2);
  c_side_beta_->setValue(coefs->c_side_beta);
  c_side_p_->setValue(coefs->c_side_p);
  c_side_r_->setValue(coefs->c_side_r);
  c_roll_beta_->setValue(coefs->c_roll_beta);
  c_roll_p_->setValue(coefs->c_roll_p);
  c_roll_r_->setValue(coefs->c_roll_r);
  c_pitch_0_->setValue(coefs->c_pitch_0);
  c_pitch_alpha_->setValue(coefs->c_pitch_alpha);
  c_pitch_abs_beta_->setValue(coefs->c_pitch_abs_beta);
  c_pitch_alpha_rate_->setValue(coefs->c_pitch_alpha_rate);
  c_pitch_q_->setValue(coefs->c_pitch_q);
  c_yaw_beta_->setValue(coefs->c_yaw_beta);
  c_yaw_p_->setValue(coefs->c_yaw_p);
  c_yaw_r_->setValue(coefs->c_yaw_r);

  qt::qInfoBox(this, "Coefficients have been loaded successfully.");
}
}  // namespace tobas::gui::sa::fw
