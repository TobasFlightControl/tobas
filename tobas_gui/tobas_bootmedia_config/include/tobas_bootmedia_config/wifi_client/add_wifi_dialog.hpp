// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QLineEdit>

#include <tobas_qt_tools/widgets/password_edit.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_wpa_supplicant/key_mgmt.hpp>

namespace tobas
{
namespace gui
{
namespace bm
{
class AddWifiDialog : public QDialog
{
  Q_OBJECT

  using self = AddWifiDialog;
  using super = QDialog;

public:
  explicit AddWifiDialog(QWidget* parent);

  QString getKeyMgmt() const;
  QString getSsid() const;
  QString getPsk() const;
  int getPriority() const;
  bool getHidden() const;

private:
  QComboBox* key_mgmt_;
  QLineEdit* ssid_;
  qt::PasswordEdit* psk_;
  qt::SpinBox* priority_;
  QCheckBox* hidden_;

  QDialogButtonBox* btn_box_;

  void enableOkButton(bool enable);

private Q_SLOTS:
  void onChanged();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
