// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class SaveMissionDialog : public QFileDialog
{
  Q_OBJECT

public:
  explicit SaveMissionDialog(QWidget* _parent, const QString& _dir);

protected:
  bool eventFilter(QObject* _obj, QEvent* _event) override;

private Q_SLOTS:
  void onLineEditTextChanged();

private:
  QPushButton* save_button_;
  QLineEdit* line_edit_;
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
