// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/add_command_dialog.hpp"

#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <magic_enum.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
AddCommandDialog::AddCommandDialog(QWidget* parent) : super(parent)
{
  setWindowTitle("Add Command");

  command_list_ = new qt::ListWidget();
  command_list_->setSelectionMode(QListWidget::SingleSelection);
  for (const auto cmd : magic_enum::enum_values<mission::Type>()) {
    command_list_->addItem(commandToText(cmd));
  }

  const auto cancel_button = new QPushButton("Cancel");
  const auto ok_button = new QPushButton("OK");
  ok_button->setDefault(true);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(cancel_button);
  cols->addWidget(ok_button);

  const auto rows = new QVBoxLayout();
  rows->addWidget(command_list_);
  rows->addLayout(cols);

  setLayout(rows);

  // Connection
  connect(command_list_, &QListWidget::itemDoubleClicked, this, &self::onItemDoubleClicked);
  connect(cancel_button, &QPushButton::clicked, this, &self::reject);
  connect(ok_button, &QPushButton::clicked, this, &self::onOkClicked);
}

mission::Type AddCommandDialog::selectedCommand() const
{
  return selected_command_;
}

void AddCommandDialog::acceptWithItem(QListWidgetItem* item)
{
  selected_command_ = textToCommand(item->text());
  accept();
}

void AddCommandDialog::onItemDoubleClicked(QListWidgetItem* item)
{
  acceptWithItem(item);
}

void AddCommandDialog::onOkClicked()
{
  const auto cur_item = command_list_->currentItem();
  if (!cur_item) {
    return;
  }

  acceptWithItem(cur_item);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
