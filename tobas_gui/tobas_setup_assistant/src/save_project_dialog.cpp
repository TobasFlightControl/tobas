// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/save_project_dialog.hpp"

#include <QDialogButtonBox>
#include <QDir>
#include <QEvent>
#include <QGridLayout>
#include <QKeyEvent>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_ros2_tools/package.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
SaveProjectDialog::SaveProjectDialog(QWidget* parent, const QString& dir, const QString& dflt_name)
  : super(parent, "Save Tobas Project", dir)
{
  setOptions(ShowDirsOnly | DontUseNativeDialog);  // Use the Qt dialog for custom settings.
  setAcceptMode(AcceptSave);  // Note that the Save button disappears if `setFileMode(Directory)` is used.
  setFilter(QDir::AllDirs | QDir::Hidden | QDir::NoDotAndDotDot);
  setLabelText(FileName, "Project name:");
  setDefaultSuffix("TBS");
  selectFile(dflt_name);

  // Get the save button.
  const auto button_box = findChild<QDialogButtonBox*>("buttonBox");
  save_button_ = button_box->button(QDialogButtonBox::Save);

  // Get the file name.
  proj_name_ = findChild<QLineEdit*>("fileNameEdit");
  proj_name_->installEventFilter(this);

  // Insert the warning text at the bottom of the layout.
  warn_text_ = new qt::Label();
  warn_text_->setTextColor(Qt::red);
  const auto grid = qt::qPointerCast<QGridLayout>(layout());
  grid->addWidget(warn_text_, grid->rowCount(), 0, 1, grid->columnCount());

  // Check save availability whenever the path changes.
  connect(proj_name_, &QLineEdit::textChanged, this, &self::onProjectPathChanged);
  connect(this, &super::directoryEntered, this, &self::onProjectPathChanged);

  // Initial check.
  onProjectPathChanged();
}

bool SaveProjectDialog::eventFilter(QObject* obj, QEvent* event)
{
  if (obj == proj_name_) {
    // Allow this only when the save button is enabled.
    if (event->type() == QEvent::KeyPress) {
      const auto key_event = static_cast<QKeyEvent*>(event);
      if (key_event->key() == Qt::Key_Return || key_event->key() == Qt::Key_Enter) {
        if (save_button_->isEnabled()) {
          accept();
        }
      }
    }
  }

  return super::eventFilter(obj, event);
}

void SaveProjectDialog::onProjectPathChanged()
{
  const auto dir = directory().absolutePath();
  const auto proj_name = proj_name_->text();

  // Must be under the home directory.
  if (!dir.startsWith(QDir::homePath())) {
    warn_text_->setText("The Tobas project must be located under your home directory.");
    save_button_->setEnabled(false);
    return;
  }

  // Must be under the src directory.
  if (!dir.contains("/src/") && !dir.endsWith("/src")) {
    warn_text_->setText("The Tobas project must be located under a \"src\" directory.");
    save_button_->setEnabled(false);
    return;
  }

  // A project cannot be created inside another project.
  if (dir.contains(cmn::kProjectExtension + QString("/")) || dir.endsWith(cmn::kProjectExtension)) {
    warn_text_->setText("A project cannot be created inside another project.");
    save_button_->setEnabled(false);
    return;
  }

  // A file name must be set.
  if (proj_name.isEmpty()) {
    warn_text_->setText("Please specify a project name.");
    save_button_->setEnabled(false);
    return;
  }

  // The package name without the extension must follow ROS conventions.
  const auto pkg_name = QFileInfo(proj_name).completeBaseName();
  if (!ros2::isValidPackageName(pkg_name.toStdString())) {
    warn_text_->setText("Project name is invalid. It must match: ^[a-z][a-z0-9_]*$");
    save_button_->setEnabled(false);
    return;
  }

  // If an extension is set, it must be the specified extension.
  if (proj_name.contains('.')) {
    if (!proj_name.endsWith(cmn::kProjectExtension)) {
      warn_text_->setText("Invalid project extension.");
      save_button_->setEnabled(false);
      return;
    }

    if (qt::getBaseName(proj_name).isEmpty()) {
      warn_text_->setText("The base name of the project is empty.");
      save_button_->setEnabled(false);
      return;
    }
  }

  warn_text_->clear();
  save_button_->setEnabled(true);
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
