// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDialog>
#include <QDialogButtonBox>
#include <QPlainTextEdit>

#include <tobas_ssh_authkey/data.hpp>

namespace tobas
{
namespace gui
{
namespace bm
{
class AddSshKeyDialog : public QDialog
{
  Q_OBJECT

  using self = AddSshKeyDialog;
  using super = QDialog;

public:
  explicit AddSshKeyDialog(QWidget* parent);

  QString getKey() const;

private:
  QPlainTextEdit* key_;
  QDialogButtonBox* btn_box_;

  bool isAcceptable() const;
  void enableOkButton(bool enable);

private Q_SLOTS:
  void onTextChanged();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
