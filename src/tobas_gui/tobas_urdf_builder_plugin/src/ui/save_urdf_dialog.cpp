// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/ui/save_urdf_dialog.hpp"

#include <QDialogButtonBox>
#include <QEvent>
#include <QKeyEvent>

#include <tobas_qt_tools/path.hpp>

namespace tobas::gui::ub::ui
{
SaveUrdfDialog::SaveUrdfDialog(QWidget* _parent, const QString& _dir)
  : QFileDialog(_parent, "Save URDF", _dir, "URDF (*.urdf);;All Files (*)")
{
  setAcceptMode(QFileDialog::AcceptSave);
  setDefaultSuffix("urdf");
  setOption(QFileDialog::DontUseNativeDialog, true);

  const auto button_box = findChild<QDialogButtonBox*>("buttonBox");
  save_button_ = button_box->button(QDialogButtonBox::Save);

  line_edit_ = findChild<QLineEdit*>("fileNameEdit");
  line_edit_->installEventFilter(this);
  connect(line_edit_, &QLineEdit::textChanged, this, &SaveUrdfDialog::onLineEditTextChanged);
}

bool SaveUrdfDialog::eventFilter(QObject* _obj, QEvent* _event)
{
  if (_obj == line_edit_ && _event->type() == QEvent::KeyPress) {
    const auto key_event = static_cast<QKeyEvent*>(_event);
    if (key_event->key() == Qt::Key_Return || key_event->key() == Qt::Key_Enter) {
      // Only accept if the save button is enabled.
      if (save_button_->isEnabled()) {
        accept();
      }

      // Consume the event.
      return true;
    }
  }

  return QFileDialog::eventFilter(_obj, _event);
}

void SaveUrdfDialog::onLineEditTextChanged()
{
  // Enable the save button only if the file name is valid.
  const auto file_name = line_edit_->text();
  if (file_name.contains('.')) {
    save_button_->setEnabled(file_name.endsWith(".urdf") && !qt::getBaseName(file_name).isEmpty());
  }
  else {
    save_button_->setEnabled(file_name.length() > 0);
  }
}
}  // namespace tobas::gui::ub::ui
