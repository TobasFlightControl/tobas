// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <QDialog>

#include "../view_model/link_view_model.hpp"

namespace Ui
{
class AddLinkDialogUI;
using AddLinkDialogUIPtr = std::shared_ptr<AddLinkDialogUI>;
}  // namespace Ui

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ui
{
class UrdfBuilderPanel;

class AddLinkDialog : public QDialog
{
  Q_OBJECT

  using self = AddLinkDialog;
  using super = QDialog;

public:
  explicit AddLinkDialog(UrdfBuilderPanel* _main, const QStringList& _link_names, view_model::LinkViewModel& _link_vm);

private Q_SLOTS:
  void onLinkNameLineEditTextChanged(const QString& _text);
  void onJointNameLineEditTextChanged(const QString& _text);
  void onJointParentComboBoxIndexChanged(int _index);

private:
  UrdfBuilderPanel* main_;
  Ui::AddLinkDialogUIPtr ui_;
  view_model::LinkViewModel& link_vm_;

  void checkValidity();
  void enableOkButton(bool _enable);
};
}  // namespace ui
}  // namespace ub
}  // namespace gui
}  // namespace tobas
