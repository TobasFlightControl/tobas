// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/preset.hpp"

#include <rapidcsv.h>

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <QCollator>
#include <QDebug>
#include <QGridLayout>
#include <QMap>
#include <QRadioButton>
#include <QVBoxLayout>
#include <ament_index_cpp/get_package_share_path.hpp>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "tobas_setup_assistant/constants.hpp"
#include "tobas_setup_assistant/rapidcsv.hpp"
#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/util.hpp"

namespace fs = std::filesystem;

namespace tobas::gui::sa::propulsion::electric
{
namespace
{
constexpr char kDataPathKey[] = "data_path";
constexpr int kModelColumns = 3;

fs::path dataDir()
{
  return ament_index_cpp::get_package_share_path(kPackageName) / "data";
}
}  // namespace

AerodynamicsWidget_Preset::AerodynamicsWidget_Preset(const PropellerWidget* _propeller) : propeller_(_propeller)
{
  category_name_ = new qt::ComboBox();
  data_lists_ = new qt::StackedWidget();

  const auto data_dir = dataDir();
  addCategories(data_dir / "thrust_stand", kThrustStand);
  addCategories(data_dir / "uiuc", kUiuc);

  // Layout
  const auto category_form = new qt::FormLayout();
  category_form->addRow("Category", category_name_);

  const auto root_rows = new QVBoxLayout(this);
  root_rows->addLayout(category_form, 0);
  root_rows->addWidget(data_lists_, 1);

  // Connection
  connect(category_name_, &qt::ComboBox::currentIndexChanged, data_lists_, &QStackedWidget::setCurrentIndex);
}

void AerodynamicsWidget_Preset::setToDefaults()
{
  for (const auto& category : data_categories_) {
    category.buttons->button(0)->setChecked(true);
  }

  selectData("thrust_stand/dji/9450.csv");
}

const char* AerodynamicsWidget_Preset::name() const
{
  return "Select Propeller Model";
}

const char* AerodynamicsWidget_Preset::description() const
{
  return "Select a bundled dataset that matches your propeller to estimate its aerodynamic constants. "
         "Use a measurement-based method if no suitable dataset is available.";
}

bool AerodynamicsWidget_Preset::isValid()
{
  const auto category_idx = category_name_->currentIndex();
  if (category_idx < 0 || data_categories_.at(category_idx).buttons->checkedId() < 0) {
    qt::qWarnBox(this, "Please select a propeller model.");
    return false;
  }

  if (!fs::is_regular_file(getCurrentDataPath())) {
    qt::qWarnBox(this, "The selected propeller dataset is no longer available.");
    return false;
  }

  return true;
}

void AerodynamicsWidget_Preset::copyFrom(const AerodynamicsWidget_Base* _src)
{
  const auto derived = qt::qConstPointerCast<AerodynamicsWidget_Preset>(_src);
  for (size_t i = 0; i < data_categories_.size(); ++i) {
    const auto selected_id = derived->data_categories_.at(i).buttons->checkedId();
    data_categories_.at(i).buttons->button(selected_id)->setChecked(true);
  }
  category_name_->setCurrentIndex(derived->category_name_->currentIndex());
}

YAML::Node AerodynamicsWidget_Preset::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kDataPathKey] = getCurrentDataPath().lexically_relative(dataDir()).generic_string();

  return node;
}

void AerodynamicsWidget_Preset::load(const YAML::Node& _node)
{
  selectData(_node[kDataPathKey].as<std::string>());
}

void AerodynamicsWidget_Preset::addCategories(const fs::path& _directory, DataType _type)
{
  // Use familiar manufacturer labels, falling back to the directory name for unknown manufacturers.
  static const QMap<QString, QString> manufacturer_names{
    { "aeronaut", "Aeronaut" }, { "apc", "APC" },     { "dji", "DJI" },         { "gemfan", "Gemfan" },
    { "hqprop", "HQProp" },     { "tarot", "Tarot" }, { "t_hobby", "T-Hobby" }, { "t_motor", "T-Motor" },
  };

  // Each manufacturer directory under this data source becomes a category.
  // Sort the directories so discovery does not depend on filesystem iteration order.
  std::vector<fs::path> directories;
  for (const auto& entry : fs::directory_iterator(_directory)) {
    if (entry.is_directory()) {
      directories.push_back(entry.path());
    }
  }
  std::ranges::sort(directories);

  for (const auto& directory : directories) {
    // Collect CSV datasets and omit categories that have no selectable models.
    std::vector<fs::path> paths;
    for (const auto& entry : fs::directory_iterator(directory)) {
      if (entry.is_regular_file() && entry.path().extension() == ".csv") {
        paths.push_back(entry.path());
      }
    }
    if (paths.empty()) {
      continue;
    }

    // Sort model names naturally, comparing embedded numbers by their numeric values.
    QCollator collator;
    collator.setNumericMode(true);
    std::ranges::sort(
      paths,
      [&collator](const auto& _lhs, const auto& _rhs)
      { return collator.compare(QString::fromStdString(_lhs.stem()), QString::fromStdString(_rhs.stem())) < 0; });

    // Keep one selection per category. Button IDs index the corresponding CSV paths.
    // Fill the grid from left to right, then continue on the next row.
    const auto buttons = new QButtonGroup(this);
    const auto grid = new QGridLayout();
    grid->setAlignment(Qt::AlignTop);
    for (int i = 0; i < kModelColumns; ++i) {
      grid->setColumnStretch(i, 1);
    }
    for (int i = 0; i < static_cast<int>(paths.size()); ++i) {
      const auto button = new QRadioButton(QString::fromStdString(paths.at(i).stem()));
      buttons->addButton(button, i);
      grid->addWidget(button, i / kModelColumns, i % kModelColumns);
    }
    buttons->button(0)->setChecked(true);

    // Give each category a scrollable page so long lists fit within the available height.
    const auto scroll = new qt::ScrollArea();
    scroll->setLayout(grid);
    scroll->setBackgroundTransparent();

    // Resolve the manufacturer's display name from the directory name.
    // Warn and use the directory name as-is if no display name is registered.
    const auto manufacturer = QString::fromStdString(directory.filename());
    QString category_name;
    if (const auto it = manufacturer_names.find(manufacturer); it != manufacturer_names.end()) {
      category_name = it.value();
    }
    else {
      qWarning() << "No display name is defined for propeller manufacturer" << manufacturer;
      category_name = manufacturer;
    }

    // Find the alphabetical position among all categories, including other data sources.
    int category_idx = 0;
    while (category_idx < category_name_->count() &&
           QString::compare(category_name_->itemText(category_idx), category_name, Qt::CaseInsensitive) < 0) {
      ++category_idx;
    }

    // Use the same index for the selector, page, and dataset metadata to keep them in sync.
    data_lists_->insertWidget(category_idx, scroll);
    category_name_->insertItem(category_idx, category_name);
    data_categories_.insert(data_categories_.begin() + category_idx, { _type, std::move(paths), buttons });
  }
}

void AerodynamicsWidget_Preset::selectData(const std::string& _name)
{
  for (int i = 0; i < static_cast<int>(data_categories_.size()); ++i) {
    const auto& category = data_categories_.at(i);
    for (int j = 0; j < static_cast<int>(category.paths.size()); ++j) {
      const auto& path = category.paths.at(j);
      if (_name == path.lexically_relative(dataDir()).generic_string()) {
        category.buttons->button(j)->setChecked(true);
        category_name_->setCurrentIndex(i);
        return;
      }
    }
  }

  throw std::runtime_error("Unknown propeller dataset: " + _name);
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
  const auto& category = data_categories_.at(category_name_->currentIndex());
  return category.paths.at(category.buttons->checkedId());
}

AerodynamicsWidget_Preset::DataType AerodynamicsWidget_Preset::getCurrentDataType() const
{
  return data_categories_.at(category_name_->currentIndex()).type;
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
AerodynamicsWidget_Preset::parseThrustStandData(const fs::path& _path) const
{
  // Load CSV.
  const auto doc = csv::load(_path);

  // Read data.
  const auto rpms = eigen::fromStdVector(doc.GetColumn<double>(kRpmColName));
  const auto thrusts = eigen::fromStdVector(doc.GetColumn<double>(kThrustColName));
  const auto torques = eigen::fromStdVector(doc.GetColumn<double>(kTorqueColName));

  return { rpms, thrusts, torques };
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
AerodynamicsWidget_Preset::parseUiucData(const fs::path& _path) const
{
  // Load CSV.
  const auto doc = csv::load(_path);

  // Read data.
  const auto rpms = eigen::fromStdVector(doc.GetColumn<double>(kRpmColName));
  const auto cts = eigen::fromStdVector(doc.GetColumn<double>(kUiucCtColName));
  const auto cps = eigen::fromStdVector(doc.GetColumn<double>(kUiucCpColName));

  return { rpms, cts, cps };
}
}  // namespace tobas::gui::sa::propulsion::electric
