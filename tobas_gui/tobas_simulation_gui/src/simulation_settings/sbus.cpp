// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/simulation_settings/sbus.hpp"

#include <QFormLayout>
#include <QSet>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sim
{
SbusWidget::SbusWidget() : dir_(kDirPath)
{
  device_names_ = new qt::ComboBox();

  // Layout
  const auto form = new QFormLayout();
  form->addRow("Device", device_names_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(new qt::Label("S.BUS", cmn::kLabelPSize, QFont::Bold));
  rows->addLayout(form);

  setLayout(rows);

  // Connection
  connect(&scan_timer_, &QTimer::timeout, this, &self::onScanTimerTimeout);

  scan_timer_.start(1s);
}

fs::path SbusWidget::devicePath() const
{
  if (device_names_->count() == 0) {
    return {};
  }

  const auto device_name = device_names_->currentText().toStdString();
  if (device_name.empty()) {
    return {};
  }

  return fs::path(kDirPath) / device_name;
}

void SbusWidget::onScanTimerTimeout()
{
  if (!dir_.exists()) {
    device_names_->clear();
    return;
  }

  const QSignalBlocker block(device_names_);

  // Enumerate new device names.
  QSet<QString> new_device_names;
  for (const auto& entry : dir_.entryInfoList(QDir::NoDotAndDotDot | QDir::Files | QDir::System)) {
    new_device_names.insert(entry.fileName());
  }

  // Provide an option to not start the S.BUS driver.
  if (device_names_->count() == 0) {
    device_names_->addItem("");
  }

  // Collect missing devices first because deleting them inside the loop invalidates iterators.
  QSet<QString> removed_device_names;
  for (int i = 1; i < device_names_->count(); ++i) {  // Index 0 is an empty string.
    const auto cur_device_name = device_names_->itemText(i);
    if (!new_device_names.contains(cur_device_name)) {
      removed_device_names.insert(cur_device_name);
    }
  }

  // Remove missing devices from the choices.
  for (const auto& removed_device_name : removed_device_names) {
    device_names_->removeText(removed_device_name);
  }

  // Add new devices to the choices.
  for (const auto& new_device_name : new_device_names) {
    if (!device_names_->contains(new_device_name)) {
      device_names_->addItem(new_device_name);

      // Automatically select the first added device.
      if (device_names_->count() == 2) {
        device_names_->setCurrentIndex(1);
      }
    }
  }
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
