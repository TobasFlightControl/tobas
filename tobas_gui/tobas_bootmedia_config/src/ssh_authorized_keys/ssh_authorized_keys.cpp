// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/ssh_authorized_keys/ssh_authorized_keys.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <filesystem>

#include <tobas_path_tools/core.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ssh_authkey/export.hpp>
#include <tobas_ssh_authkey/parse.hpp>
#include <tobas_ssh_authkey/prettify.hpp>
#include <tobas_std_tools/error.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_string_tools/stream.hpp>

#include "tobas_bootmedia_config/constants.hpp"
#include "tobas_bootmedia_config/ssh_authorized_keys/add_key_dialog.hpp"

namespace fs = std::filesystem;

namespace tobas::gui::bm
{
namespace
{
fs::path authorizedKeysPath()
{
  return fs::path(kHomePath) / ".ssh/authorized_keys";
}
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

  if (!updatePermissions()) {
    return false;
  }

  // Enable edit buttons.
  add_button_->setEnabled(true);
  remove_button_->setEnabled(true);
  clear_button_->setEnabled(true);

  return true;
}

bool SshAuthorizedKeysWidget::addKey(const ssh::ak::Data& _key)
{
  const auto key_disp = ssh::ak::prettify(_key);
  if (!key_disp) {
    qt::qErrorBox(
      this, "Failed to convert the SSH public key to a display string: " + QString::fromStdString(key_disp.error()));
    return false;
  }

  keys_.push_back(_key);
  list_->addItem(QString::fromStdString(*key_disp));

  return true;
}

bool SshAuthorizedKeysWidget::updatePermissions()
{
  // Use the media user's numeric UID/GID rather than the host's user database.
  struct stat home_stat = {};
  if (::stat(kHomePath, &home_stat) < 0) {
    qt::qErrorBox(
      this, "Failed to read ownership of " + QString(kHomePath) + ":\n" + QString::fromStdString(st::strError()));
    return false;
  }

  const auto set_permissions = [this, &home_stat](const fs::path& _path, mode_t _mode)
  {
    const auto display_path = QString::fromStdString(_path.string());
    if (::chown(_path.c_str(), home_stat.st_uid, home_stat.st_gid) < 0) {
      qt::qErrorBox(this, "Failed to set ownership of " + display_path + ":\n" + QString::fromStdString(st::strError()));
      return false;
    }
    if (::chmod(_path.c_str(), _mode) < 0) {
      qt::qErrorBox(
        this, "Failed to set permissions of " + display_path + ":\n" + QString::fromStdString(st::strError()));
      return false;
    }
    return true;
  };

  const auto file_path = authorizedKeysPath();
  return set_permissions(file_path.parent_path(), 0700) && set_permissions(file_path, 0600);
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
  if (!updatePermissions()) {
    return false;
  }
  if (!str::writeText(file_path.string(), content)) {
    qt::qErrorBox(this, "Failed to write to " + QString::fromStdString(file_path.string()));
    return false;
  }

  qt::qInfoBox(this, "SSH authorized keys are updated successfully.");
  return true;
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
}  // namespace tobas::gui::bm
