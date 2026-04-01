// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/ssh_authorized_keys/add_key_dialog.hpp"

#include <QPushButton>
#include <QVBoxLayout>

#include "tobas_bootmedia_config/constants.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
AddSshKeyDialog::AddSshKeyDialog(QWidget* parent) : super(parent)
{
  setWindowTitle("Add New SSH Public Key");

  key_ = new QPlainTextEdit();
  const QFontMetrics fm(key_->font());
  key_->setMinimumHeight(fm.lineSpacing() * 6 + key_->frameWidth() * 2);  // ある程度の縦幅を確保

  btn_box_ = new QDialogButtonBox();
  btn_box_->setOrientation(Qt::Horizontal);
  btn_box_->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  enableOkButton(false);

  // Layout
  const auto rows = new QVBoxLayout();
  rows->addWidget(key_);
  rows->addWidget(btn_box_);

  setLayout(rows);

  // Connection
  connect(key_, &QPlainTextEdit::textChanged, this, &self::onTextChanged);
  connect(btn_box_, &QDialogButtonBox::accepted, this, &self::accept);
  connect(btn_box_, &QDialogButtonBox::rejected, this, &self::reject);
}

QString AddSshKeyDialog::getKey() const
{
  return key_->toPlainText();
}

bool AddSshKeyDialog::isAcceptable() const
{
  const auto key = getKey();
  if (key.isEmpty()) {
    return false;
  }

  return true;
}

void AddSshKeyDialog::enableOkButton(bool enable)
{
  btn_box_->button(QDialogButtonBox::Ok)->setEnabled(enable);
}

void AddSshKeyDialog::onTextChanged()
{
  enableOkButton(isAcceptable());
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
