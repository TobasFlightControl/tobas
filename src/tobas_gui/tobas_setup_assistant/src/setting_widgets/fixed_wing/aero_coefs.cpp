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

  c_lift_0_ = new qt::DoubleSpinBox();
  c_lift_0_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_lift_0_->setDecimals(kStabilityCoefDecimals);
  c_lift_0_->setSuffix(" [-]");

  c_lift_alpha_ = new qt::DoubleSpinBox();
  c_lift_alpha_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_lift_alpha_->setDecimals(kStabilityCoefDecimals);
  c_lift_alpha_->setSuffix(" [/rad]");

  c_drag_0_ = new qt::DoubleSpinBox();
  c_drag_0_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_drag_0_->setDecimals(kStabilityCoefDecimals);
  c_drag_0_->setSuffix(" [-]");

  c_drag_alpha_ = new qt::DoubleSpinBox();
  c_drag_alpha_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_drag_alpha_->setDecimals(kStabilityCoefDecimals);
  c_drag_alpha_->setSuffix(" [/rad]");

  c_side_beta_ = new qt::DoubleSpinBox();
  c_side_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_side_beta_->setDecimals(kStabilityCoefDecimals);
  c_side_beta_->setSuffix(" [/rad]");

  c_roll_beta_ = new qt::DoubleSpinBox();
  c_roll_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_roll_beta_->setDecimals(kStabilityCoefDecimals);
  c_roll_beta_->setSuffix(" [/rad]");

  c_roll_p_ = new qt::DoubleSpinBox();
  c_roll_p_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_roll_p_->setDecimals(kStabilityCoefDecimals);
  c_roll_p_->setSuffix(" [s/rad]");

  c_roll_r_ = new qt::DoubleSpinBox();
  c_roll_r_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_roll_r_->setDecimals(kStabilityCoefDecimals);
  c_roll_r_->setSuffix(" [s/rad]");

  c_pitch_0_ = new qt::DoubleSpinBox();
  c_pitch_0_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_0_->setDecimals(kStabilityCoefDecimals);
  c_pitch_0_->setSuffix(" [-]");

  c_pitch_alpha_ = new qt::DoubleSpinBox();
  c_pitch_alpha_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_alpha_->setDecimals(kStabilityCoefDecimals);
  c_pitch_alpha_->setSuffix(" [/rad]");

  c_pitch_abs_beta_ = new qt::DoubleSpinBox();
  c_pitch_abs_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_abs_beta_->setDecimals(kStabilityCoefDecimals);
  c_pitch_abs_beta_->setSuffix(" [/rad]");

  c_pitch_alpha_rate_ = new qt::DoubleSpinBox();
  c_pitch_alpha_rate_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_alpha_rate_->setDecimals(kStabilityCoefDecimals);
  c_pitch_alpha_rate_->setSuffix(" [s/rad]");

  c_pitch_q_ = new qt::DoubleSpinBox();
  c_pitch_q_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_pitch_q_->setDecimals(kStabilityCoefDecimals);
  c_pitch_q_->setSuffix(" [s/rad]");

  c_yaw_beta_ = new qt::DoubleSpinBox();
  c_yaw_beta_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_yaw_beta_->setDecimals(kStabilityCoefDecimals);
  c_yaw_beta_->setSuffix(" [/rad]");

  c_yaw_p_ = new qt::DoubleSpinBox();
  c_yaw_p_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_yaw_p_->setDecimals(kStabilityCoefDecimals);
  c_yaw_p_->setSuffix(" [s/rad]");

  c_yaw_r_ = new qt::DoubleSpinBox();
  c_yaw_r_->setRange(-kMaxStabilityCoef, kMaxStabilityCoef);
  c_yaw_r_->setDecimals(kStabilityCoefDecimals);
  c_yaw_r_->setSuffix(" [s/rad]");

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(load_button);
  button_cols->addStretch(1);

  form_ = new qt::FormLayout();
  form_->addRow("c_lift_0", c_lift_0_);
  form_->addRow("c_lift_alpha", c_lift_alpha_);
  form_->addRow("c_drag_0", c_drag_0_);
  form_->addRow("c_drag_alpha", c_drag_alpha_);
  form_->addRow("c_side_beta", c_side_beta_);
  form_->addRow("c_roll_beta", c_roll_beta_);
  form_->addRow("c_roll_p", c_roll_p_);
  form_->addRow("c_roll_r", c_roll_r_);
  form_->addRow("c_pitch_0", c_pitch_0_);
  form_->addRow("c_pitch_alpha", c_pitch_alpha_);
  form_->addRow("c_pitch_abs_beta", c_pitch_abs_beta_);
  form_->addRow("c_pitch_alpha_rate", c_pitch_alpha_rate_);
  form_->addRow("c_pitch_q", c_pitch_q_);
  form_->addRow("c_yaw_beta", c_yaw_beta_);
  form_->addRow("c_yaw_p", c_yaw_p_);
  form_->addRow("c_yaw_r", c_yaw_r_);

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
  c_side_beta_->setValue(-0.3073);
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

double AerodynamicsCoefficientsWidget::c_side_beta() const
{
  return c_side_beta_->value();
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
  c_side_beta_->setValue(coefs->c_side_beta);
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
