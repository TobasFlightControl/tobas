// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/save_mission_dialog.hpp"

#include <QDialogButtonBox>
#include <QEvent>
#include <QKeyEvent>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/path.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
SaveMissionDialog::SaveMissionDialog(QWidget* parent, const QString& dir)
  : QFileDialog(parent, "Save Mission", dir, "Mission (*.mission);;All Files (*)")
{
  setAcceptMode(QFileDialog::AcceptSave);
  setDefaultSuffix("mission");
  setOption(QFileDialog::DontUseNativeDialog, true);

  const auto button_box = findChild<QDialogButtonBox*>("buttonBox");
  save_button_ = button_box->button(QDialogButtonBox::Save);

  line_edit_ = findChild<QLineEdit*>("fileNameEdit");
  line_edit_->installEventFilter(this);
  connect(line_edit_, &QLineEdit::textChanged, this, &SaveMissionDialog::onLineEditTextChanged);
}

bool SaveMissionDialog::eventFilter(QObject* obj, QEvent* event)
{
  if (obj == line_edit_ && event->type() == QEvent::KeyPress) {
    const auto key_event = static_cast<QKeyEvent*>(event);
    if (key_event->key() == Qt::Key_Return || key_event->key() == Qt::Key_Enter) {
      // Only accept if the save button is enabled
      if (save_button_->isEnabled()) {
        accept();
      }

      // Consume the event
      return true;
    }
  }

  return QFileDialog::eventFilter(obj, event);
}

void SaveMissionDialog::onLineEditTextChanged()
{
  // Enable the save button only if the file name is valid
  const auto file_name = line_edit_->text();
  if (file_name.contains('.')) {
    save_button_->setEnabled(file_name.endsWith(cmn::kMissionExtension) && !qt::getBaseName(file_name).isEmpty());
  }
  else {
    save_button_->setEnabled(file_name.length() > 0);
  }
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
