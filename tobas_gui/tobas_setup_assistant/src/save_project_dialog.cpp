#include "tobas_setup_assistant/save_project_dialog.hpp"

#include <QDebug>
#include <QDialogButtonBox>
#include <QEvent>
#include <QKeyEvent>

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_ros2_tools/util.hpp>

namespace gui
{
namespace sa
{
SaveProjectDialog::SaveProjectDialog(QWidget* parent, const QString& dir, const QString& dflt_name)
  : super(parent, "Save Tobas Project", dir, "Tobas Project (*.TBS);;All Files (*)")
{
  setAcceptMode(QFileDialog::AcceptSave);
  setOption(QFileDialog::DontUseNativeDialog, true);
  setDefaultSuffix("TBS");
  selectFile(dflt_name);

  const auto button_box = findChild<QDialogButtonBox*>("buttonBox");
  save_button_ = button_box->button(QDialogButtonBox::Save);

  file_name_ = findChild<QLineEdit*>("fileNameEdit");
  file_name_->installEventFilter(this);

  connect(this, &super::directoryEntered, this, &self::onFilePathChanged);
  connect(file_name_, &QLineEdit::textChanged, this, &self::onFilePathChanged);
}

bool SaveProjectDialog::eventFilter(QObject* obj, QEvent* event)
{
  if (obj == file_name_ && event->type() == QEvent::KeyPress) {
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

  return super::eventFilter(obj, event);
}

void SaveProjectDialog::onFilePathChanged()
{
  const auto dir = directory().absolutePath();
  const auto file_name = file_name_->text();

  // ホームディレクトリ以下でなければならない
  if (!dir.startsWith(ros2::getHomeDir())) {
    qWarning() << "The directory is not under the home directory.";
    save_button_->setEnabled(false);
    return;
  }

  // srcディレクトリ以下でなければならない
  if (!dir.contains("/src/") && !dir.endsWith("/src")) {
    qWarning() << "The directory is not under a \"src\" directory.";
    save_button_->setEnabled(false);
    return;
  }

  // ファイル名が設定されていなければならない
  if (file_name.isEmpty()) {
    qWarning() << "The file name is empty.";
    save_button_->setEnabled(false);
    return;
  }

  // 拡張子が設定されている場合は決められた拡張子でなければならない
  if (file_name.contains('.')) {
    if (!file_name.endsWith(tobas::kProjectExtension)) {
      qWarning() << "Invalid extension.";
      save_button_->setEnabled(false);
      return;
    }

    if (qt::getBaseName(file_name).isEmpty()) {
      qWarning() << "The base name of the file is empty.";
      save_button_->setEnabled(false);
      return;
    }
  }

  save_button_->setEnabled(true);
}
}  // namespace sa
}  // namespace gui
