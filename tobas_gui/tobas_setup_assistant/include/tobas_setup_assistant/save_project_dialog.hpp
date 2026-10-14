// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFileDialog>
#include <QLineEdit>
#include <QPushButton>

#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
class SaveProjectDialog : public QFileDialog
{
  Q_OBJECT

  using self = SaveProjectDialog;
  using super = QFileDialog;

public:
  explicit SaveProjectDialog(QWidget* _parent, const QString& _dir, const QString& _dflt_name);

protected:
  bool eventFilter(QObject* _obj, QEvent* _event) override;

private:
  QPushButton* save_button_;
  QLineEdit* proj_name_;
  qt::Label* warn_text_;

private Q_SLOTS:
  void onProjectPathChanged();
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
