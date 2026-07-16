// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/wifi_client/add_wifi_dialog.hpp"

#include <QFormLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <magic_enum/magic_enum.hpp>

#include "tobas_bootmedia_config/constants.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
AddWifiDialog::AddWifiDialog(QWidget* parent) : super(parent)
{
  setWindowTitle("Add New Network");

  key_mgmt_ = new QComboBox();
  for (const auto val : magic_enum::enum_values<wpa::KeyMgmt>()) {
    key_mgmt_->addItem(QString::fromStdString(wpa::labelFromEnum(val)));
  }
  key_mgmt_->setCurrentText(QString::fromStdString(wpa::labelFromEnum(wpa::KeyMgmt::WPA_PSK)));

  ssid_ = new QLineEdit();

  psk_ = new qt::PasswordEdit();

  priority_ = new qt::SpinBox();
  priority_->setMinimum(0);
  priority_->setMaximum(99);
  priority_->setValue(0);

  hidden_ = new QCheckBox();

  btn_box_ = new QDialogButtonBox();
  btn_box_->setOrientation(Qt::Horizontal);
  btn_box_->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  enableOkButton(false);

  // Layout
  const auto form = new QFormLayout();
  form->addRow("Key Management", key_mgmt_);
  form->addRow("SSID", ssid_);
  form->addRow("PSK", psk_);
  form->addRow("Priority", priority_);
  form->addRow("Hidden", hidden_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(form);
  rows->addWidget(btn_box_);

  setLayout(rows);

  // Connection
  connect(key_mgmt_, &QComboBox::currentTextChanged, this, &self::onChanged);
  connect(ssid_, &QLineEdit::textChanged, this, &self::onChanged);
  connect(psk_, &QLineEdit::textChanged, this, &self::onChanged);
  connect(btn_box_, &QDialogButtonBox::accepted, this, &self::accept);
  connect(btn_box_, &QDialogButtonBox::rejected, this, &self::reject);
}

QString AddWifiDialog::getKeyMgmt() const
{
  return key_mgmt_->currentText();
}

QString AddWifiDialog::getSsid() const
{
  return ssid_->text();
}

QString AddWifiDialog::getPsk() const
{
  return psk_->text();
}

int AddWifiDialog::getPriority() const
{
  return priority_->value();
}

bool AddWifiDialog::getHidden() const
{
  return hidden_->isChecked();
}

void AddWifiDialog::enableOkButton(bool enable)
{
  btn_box_->button(QDialogButtonBox::Ok)->setEnabled(enable);
}

void AddWifiDialog::onChanged()
{
  // Check SSID.
  const auto ssid = getSsid();
  if (ssid.isEmpty()) {
    enableOkButton(false);
    return;
  }

  // Check PSK.
  if (getKeyMgmt().toStdString() == wpa::labelFromEnum(wpa::KeyMgmt::NONE)) {
    psk_->setEnabled(false);
  }
  else {
    psk_->setEnabled(true);
    const auto psk = getPsk();
    if (psk.length() < kWpaPskMinLength) {
      enableOkButton(false);
      return;
    }
  }

  enableOkButton(true);
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
