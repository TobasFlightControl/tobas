// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/simulation_settings/world/custom_world.hpp"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sim
{
namespace
{
constexpr char kLastOpenedDirKey[] = "simulation_settings/world/custom_world/last_opened_dir";
}  // namespace

CustomWorldWidget::CustomWorldWidget()
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  file_text_ = new QLineEdit();
  file_text_->setReadOnly(true);
  file_text_->setFocusPolicy(Qt::NoFocus);
  cols->addWidget(file_text_);

  browse_button_ = new QPushButton("Browse");
  connect(browse_button_, &QPushButton::clicked, this, &self::onBrowseButtonClicked);
  cols->addWidget(browse_button_);
}

fs::path CustomWorldWidget::worldPath() const
{
  return file_text_->text().toStdString();
}

void CustomWorldWidget::onBrowseButtonClicked()
{
  // Get the previously opened path.
  const auto last_opened_dir = settings_store_.value(kLastOpenedDirKey, QDir::homePath()).toString();

  // Get the world path.
  const auto file_path = QFileDialog::getOpenFileName(
    this, "Select World File", last_opened_dir, "Gazebo World (*.world)", nullptr, QFileDialog::DontUseNativeDialog);

  // Return without doing anything if canceled.
  if (file_path.isEmpty()) {
    return;
  }

  // Set the path text.
  file_text_->setText(file_path);

  // Save the directory opened by the user.
  const auto par_dir = QFileInfo(file_path).absolutePath();
  settings_store_.setValue(kLastOpenedDirKey, par_dir);
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
