// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDialog>

#include <tobas_qt_tools/widgets/list_widget.hpp>

#include "./command_type.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class AddCommandDialog : public QDialog
{
  Q_OBJECT

  using self = AddCommandDialog;
  using super = QDialog;

public:
  explicit AddCommandDialog(QWidget* parent);

  Command selectedCommand() const;

private:
  qt::ListWidget* command_list_;
  Command selected_command_;

private Q_SLOTS:
  void onOkClicked();
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
