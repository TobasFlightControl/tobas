// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/thrust_stand.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/eigen.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/constants.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/util.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
AerodynamicsWidget_ThrustStand::AerodynamicsWidget_ThrustStand()
{
  data_ = new ParamGetterWidget_DoubleTable(
    "Data from thrust stand", "Select Test Data", { kRpmColName, kThrustColName, kTorqueColName });
  data_->setDecimals({ 0, 6, 6 });
  data_->setMinimum({ 1, 1e-6, 1e-6 });
  data_->table()->setColumnsWidth(kDataTableColWidth);

  rows_->addWidget(data_);
}

void AerodynamicsWidget_ThrustStand::setToDefaults()
{
  data_->table()->removeAll();
}

const char* AerodynamicsWidget_ThrustStand::name() const
{
  return "Estimate from Thrust Stand Data";
}

const char* AerodynamicsWidget_ThrustStand::description() const
{
  // Hyperlinks do not work if a newline is inserted in the text.
  return "We estimate the aerodynamic constants from data obtained through Thrust Stand experiments. "
         "For example, see "
         "<a href='https://www.tytorobotics.com/pages/series-1580-1585'>Tyto Rootics Series 1585 Thrust Stand</a>.";
}

bool AerodynamicsWidget_ThrustStand::isValid()
{
  if (data_->count() == 0) {
    qt::qWarnBox(this, "Thrust stand data is blank.");
    return false;
  }

  return true;
}

void AerodynamicsWidget_ThrustStand::copyFrom(const AerodynamicsWidget_Base* src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget_ThrustStand>(src);
  data_->setValue(derived->data_->getValue());
}

YAML::Node AerodynamicsWidget_ThrustStand::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[data_->name()] = data_->getValue();

  return node;
}

void AerodynamicsWidget_ThrustStand::load(const YAML::Node& node)
{
  data_->setValue(node[data_->name()].as<Eigen::MatrixXd>());
}

double AerodynamicsWidget_ThrustStand::motorConst() const
{
  // TODO: Remove outliers
  // TODO: Warn if the data is too far from the model (linear function).
  const auto data_mat = data_->getValue();
  const auto rpms = data_mat.col(0).eval();
  const auto thrusts = data_mat.col(1).eval();
  return motorConstFromThrustStand(rpms, thrusts);
}

double AerodynamicsWidget_ThrustStand::momentConst() const
{
  // TODO: Remove outliers
  // TODO: Warn if the data is too far from the model (linear function).
  const auto data_mat = data_->getValue();
  const auto thrusts = data_mat.col(1).eval();
  const auto torques = data_mat.col(2).eval();
  return momentConstFromThrustStand(thrusts, torques);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
