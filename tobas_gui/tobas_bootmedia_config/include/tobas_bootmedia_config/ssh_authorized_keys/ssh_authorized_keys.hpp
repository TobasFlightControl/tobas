// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_ssh_authkey/data.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class SshAuthorizedKeysWidget : public BaseConfigWidget
{
  Q_OBJECT

  using self = SshAuthorizedKeysWidget;
  using super = BaseConfigWidget;

  static constexpr int kColWidth = 300;

  static constexpr int kSsidCol = 0;
  static constexpr int kPskCol = 1;
  static constexpr int kPriorityCol = 2;
  static constexpr int kNumCols = 3;

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

  bool addKey(const ssh::ak::Data& key);
  bool writeCurrentConfig();

  static std::string authorizedKeysPath();

private Q_SLOTS:
  void onAddButtonClicked();
  void onRemoveButtonClicked();
  void onClearButtonClicked();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
