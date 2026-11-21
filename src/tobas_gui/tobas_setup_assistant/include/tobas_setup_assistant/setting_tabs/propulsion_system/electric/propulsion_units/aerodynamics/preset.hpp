// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <QButtonGroup>
#include <eigen3/Eigen/Core>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../propeller.hpp"
#include "./base.hpp"

namespace tobas::gui::sa::propulsion::electric
{
class AerodynamicsWidget_Preset : public AerodynamicsWidget_Base
{
  Q_OBJECT

  enum DataType
  {
    kThrustStand,
    kUiuc,
  };

  struct DataCategory
  {
    DataType type;
    std::vector<std::filesystem::path> paths;
    QButtonGroup* buttons;
  };

public:
  explicit AerodynamicsWidget_Preset(const PropellerWidget* _propeller);

  const char* name() const override;
  const char* description() const override;

  void setToDefaults() override;
  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* _src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  double motorConst() const override;
  double momentConst() const override;

private:
  const PropellerWidget* const propeller_;

  qt::ComboBox* category_name_;
  qt::StackedWidget* data_lists_;
  std::vector<DataCategory> data_categories_;

  void addCategories(const std::filesystem::path& _directory, DataType _type);
  void selectData(const std::string& _name);

  std::filesystem::path getCurrentDataPath() const;
  DataType getCurrentDataType() const;

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
  parseThrustStandData(const std::filesystem::path& _path) const;
  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> parseUiucData(const std::filesystem::path& _path) const;
};
}  // namespace tobas::gui::sa::propulsion::electric
