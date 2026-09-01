// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/ssh_authorized_keys/ssh_authorized_keys.hpp"

#include <tobas_path_tools/core.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ssh_authkey/export.hpp>
#include <tobas_ssh_authkey/parse.hpp>
#include <tobas_ssh_authkey/prettify.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_string_tools/stream.hpp>

#include "tobas_bootmedia_config/constants.hpp"
#include "tobas_bootmedia_config/ssh_authorized_keys/add_key_dialog.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace bm
{
namespace
{
constexpr int kColWidth = 300;

constexpr int kSsidCol = 0;
constexpr int kPskCol = kSsidCol + 1;
constexpr int kPriorityCol = kPskCol + 1;
constexpr int kNumCols = kPriorityCol + 1;
}  // namespace

SshAuthorizedKeysWidget::SshAuthorizedKeysWidget()
{
  add_button_ = new QPushButton("Add");
  remove_button_ = new QPushButton("Remove");
  clear_button_ = new QPushButton("Clear");

  add_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  remove_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  clear_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  list_ = new qt::ListWidget();
  list_->setSelectionMode(QListWidget::SingleSelection);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(add_button_);
  cols->addWidget(remove_button_);
  cols->addWidget(clear_button_);
  cols->addStretch();

  rows_->addLayout(cols);
  rows_->addWidget(list_);

  // Connection
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  connect(remove_button_, &QPushButton::clicked, this, &self::onRemoveButtonClicked);
  connect(clear_button_, &QPushButton::clicked, this, &self::onClearButtonClicked);
}

const char* SshAuthorizedKeysWidget::title() const
{
  return "Add SSH Authorized Keys";
}

void SshAuthorizedKeysWidget::reset()
{
  add_button_->setEnabled(false);
  remove_button_->setEnabled(false);
  clear_button_->setEnabled(false);

  keys_.clear();
  list_->clear();
}

bool SshAuthorizedKeysWidget::onConnected()
{
  const auto file_path = authorizedKeysPath();

  if (fs::exists(file_path)) {
    // Parse the file and get keys.
    const auto keys = ssh::ak::parseFile(authorizedKeysPath());
    if (!keys) {
      qt::qErrorBox(this, QString::fromStdString(keys.error()));
      return false;
    }

    // Reflect the current keys in the list.
    keys_.clear();
    list_->clear();
    for (const auto& key : *keys) {
      if (!addKey(key)) {
        return false;
      }
    }
  }
  else {
    // Create the file if it does not exist.
    const auto res = path::createFilePath(file_path);
    if (!res) {
      qt::qErrorBox(this, "Failed to create the authorized keys file:\n" + QString::fromStdString(res.error()));
      return false;
    }
  }

  // Enable edit buttons.
  add_button_->setEnabled(true);
  remove_button_->setEnabled(true);
  clear_button_->setEnabled(true);

  return true;
}

bool SshAuthorizedKeysWidget::addKey(const ssh::ak::Data& key)
{
  const auto key_disp = ssh::ak::prettify(key);
  if (!key_disp) {
    qt::qErrorBox(
      this, "Failed to convert the SSH public key to a display string: " + QString::fromStdString(key_disp.error()));
    return false;
  }

  keys_.push_back(key);
  list_->addItem(QString::fromStdString(*key_disp));

  return true;
}

bool SshAuthorizedKeysWidget::writeCurrentConfig()
{
  // Output all keys as text.
  std::string content;
  for (const auto& key : keys_) {
    const auto line = ssh::ak::exportLine(key);
    if (!line) {
      qt::qErrorBox(this, "Failed to export SSH key: " + QString::fromStdString(line.error()));
      return false;
    }
    content += *line + '\n';
  }

  // Write the configuration.
  const auto file_path = authorizedKeysPath();
  if (!str::writeText(file_path, content)) {
    qt::qErrorBox(this, "Failed to write to " + QString::fromStdString(file_path));
    return false;
  }

  qt::qInfoBox(this, "SSH authorized keys are updated successfully.");
  return true;
}

std::string SshAuthorizedKeysWidget::authorizedKeysPath()
{
  return std::string(kHomePath) + "/.ssh/authorized_keys";
}

void SshAuthorizedKeysWidget::onAddButtonClicked()
{
  // Get a key from the dialog.
  AddSshKeyDialog dialog(this);
  const auto result = dialog.exec();
  if (result != QDialog::Accepted) {
    return;
  }

  // Parse the key.
  const auto line = dialog.getKey().toStdString();
  const auto key = ssh::ak::parseLine(line);
  if (!key) {
    qt::qErrorBox(this, "Invalid SSH public key: " + QString::fromStdString(key.error()));
    return;
  }

  // Add the key.
  if (!addKey(*key)) {
    return;
  }

  // Apply the current configuration to the media.
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}

void SshAuthorizedKeysWidget::onRemoveButtonClicked()
{
  // Get the rows to delete.
  const auto row = list_->currentRow();
  if (row < 0) {
    qt::qWarnBox(this, "Please select the SSH key to remove.");
    return;
  }

  // Confirm before deleting the selected key.
  if (!qt::yesOrNo(this, "Are you sure you want to remove the selected key?", qt::WARN)) {
    return;
  }

  // Delete the key.
  st::eraseIndex(keys_, row);
  list_->takeItem(row);

  // Apply the current configuration to the media.
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}

void SshAuthorizedKeysWidget::onClearButtonClicked()
{
  // Confirm before deleting everything.
  if (!qt::yesOrNo(this, "Are you sure you want to remove all SSH keys?", qt::WARN)) {
    return;
  }

  // Delete all keys.
  keys_.clear();
  list_->clear();

  // Apply the current configuration to the media.
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
