// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_ssh_authkey/data.hpp>

#include "../base.hpp"

namespace tobas::gui::bm
{
class SshAuthorizedKeysWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = SshAuthorizedKeysWidget;

public:
  explicit SshAuthorizedKeysWidget();

  const char* title() const override;

  void reset() override;

  bool onConnected() override;

private:
  QPushButton* add_button_;
  QPushButton* remove_button_;
  QPushButton* clear_button_;

  std::vector<ssh::ak::Data> keys_;
  qt::ListWidget* list_;

  bool addKey(const ssh::ak::Data& _key);
  bool updatePermissions();
  bool writeCurrentConfig();

private Q_SLOTS:
  void onAddButtonClicked();
  void onRemoveButtonClicked();
  void onClearButtonClicked();
};
}  // namespace tobas::gui::bm
