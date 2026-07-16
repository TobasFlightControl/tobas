// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/preset.hpp"

#include <rapidcsv.h>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/rapidcsv.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/util.hpp"
#include "tobas_setup_assistant/util.hpp"

namespace fs = std::filesystem;

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
AerodynamicsWidget_Preset::AerodynamicsWidget_Preset(const PropellerWidget* propeller) : propeller_(propeller)
{
  data_name_ = new qt::ComboBox();

  // Add thrust data names.
  for (const auto& entry : fs::recursive_directory_iterator(thrustStandDataDir())) {
    if (entry.is_regular_file() && entry.path().extension() == ".csv") {
      const auto data_name = entry.path().stem().string();
      data_name_->addItem(QString::fromStdString(data_name), static_cast<int>(kThrustStand));
    }
  }

  // Add UIUC data names.
  for (const auto& entry : fs::recursive_directory_iterator(uiucDataDir())) {
    if (entry.is_regular_file() && entry.path().extension() == ".csv") {
      const auto data_name = entry.path().stem().string();
      data_name_->addItem(QString::fromStdString(data_name), static_cast<int>(kUiuc));
    }
  }

  // Sort data.
  data_name_->sort();

  // Default
  data_name_->setCurrentText("dji_9450");

  // Layout
  rows_->addWidget(data_name_);
  rows_->addStretch();
}

const char* AerodynamicsWidget_Preset::name() const
{
  return "Select Propeller Model";
}

const char* AerodynamicsWidget_Preset::description() const
{
  return "If your propeller appears in the list, "
         "just pick it and the correct aerodynamic parameters will be applied automatically.";
}

bool AerodynamicsWidget_Preset::isValid()
{
  return true;
}

void AerodynamicsWidget_Preset::copyFrom(const AerodynamicsWidget_Base* src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget_Preset>(src);
  data_name_->setCurrentText(derived->data_name_->currentText());
}

YAML::Node AerodynamicsWidget_Preset::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kPropellerNameKey] = data_name_->currentText();

  return node;
}

void AerodynamicsWidget_Preset::load(const YAML::Node& node)
{
  data_name_->setCurrentText(node[kPropellerNameKey].as<QString>());
}

double AerodynamicsWidget_Preset::motorConst() const
{
  const auto data_path = getCurrentDataPath();
  const auto data_type = getCurrentDataType();

  switch (data_type) {
    case kThrustStand: {
      const auto [rpms, thrusts, _] = parseThrustStandData(data_path);
      return motorConstFromThrustStand(rpms, thrusts);
    }
    case kUiuc: {
      const auto [_, cts, __] = parseUiucData(data_path);
      const auto d = propeller_->diameter();
      return motorConstFromUiuc(cts, d);
    }
    default: {
      throw;
    }
  }
}

double AerodynamicsWidget_Preset::momentConst() const
{
  const auto data_path = getCurrentDataPath();
  const auto data_type = getCurrentDataType();

  switch (data_type) {
    case kThrustStand: {
      const auto [_, thrusts, torques] = parseThrustStandData(data_path);
      return momentConstFromThrustStand(thrusts, torques);
    }
    case kUiuc: {
      const auto [_, cts, cps] = parseUiucData(data_path);
      const auto d = propeller_->diameter();
      return momentConstFromUiuc(cts, cps, d);
    }
    default: {
      throw;
    }
  }
}

fs::path AerodynamicsWidget_Preset::getCurrentDataPath() const
{
  const auto data_name = data_name_->currentText().toStdString() + ".csv";
  const auto data_type = static_cast<DataType>(data_name_->currentData().toInt());

  switch (data_type) {
    case kThrustStand: {
      return thrustStandDataDir() / data_name;
    }
    case kUiuc: {
      return uiucDataDir() / data_name;
    }
    default: {
      throw;
    }
  }
}

AerodynamicsWidget_Preset::DataType AerodynamicsWidget_Preset::getCurrentDataType() const
{
  return static_cast<DataType>(data_name_->currentData().toInt());
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
AerodynamicsWidget_Preset::parseThrustStandData(const fs::path& path) const
{
  // Load CSV.
  const auto doc = csv::load(path);

  // Read data.
  const auto rpms = eigen::fromStdVector(doc.GetColumn<double>(kRpmColName));
  const auto thrusts = eigen::fromStdVector(doc.GetColumn<double>(kThrustColName));
  const auto torques = eigen::fromStdVector(doc.GetColumn<double>(kTorqueColName));

  return { rpms, thrusts, torques };
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
AerodynamicsWidget_Preset::parseUiucData(const fs::path& path) const
{
  // Load CSV.
  const auto doc = csv::load(path);

  // Read data.
  const auto rpms = eigen::fromStdVector(doc.GetColumn<double>(kRpmColName));
  const auto cts = eigen::fromStdVector(doc.GetColumn<double>(kUiucCtColName));
  const auto cps = eigen::fromStdVector(doc.GetColumn<double>(kUiucCpColName));

  return { rpms, cts, cps };
}

fs::path AerodynamicsWidget_Preset::dataDir()
{
  return getPkgShareDir() / "data";
}

fs::path AerodynamicsWidget_Preset::thrustStandDataDir()
{
  return dataDir() / "thrust_stand";
}

fs::path AerodynamicsWidget_Preset::uiucDataDir()
{
  return dataDir() / "uiuc";
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
