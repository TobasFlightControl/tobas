#include "../../include/tobas_urdf_builder_plugin/ui/save_urdf_dialog.hpp"

#include <QDialogButtonBox>
#include <QEvent>
#include <QKeyEvent>

#include "../../include/tobas_urdf_builder_plugin/utils/string.hpp"

namespace gui
{
namespace urdf_builder
{
namespace ui
{
SaveUrdfDialog::SaveUrdfDialog(QWidget* parent, const QString& dir)
  : QFileDialog(parent, tr("Save URDF"), dir, tr("URDF (*.urdf);;All Files (*)"))
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

bool SaveUrdfDialog::eventFilter(QObject* obj, QEvent* event)
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

void SaveUrdfDialog::onLineEditTextChanged()
{
  // Enable the save button only if the file name is valid
  const auto file_name = line_edit_->text();
  if (file_name.contains('.')) {
    save_button_->setEnabled(file_name.endsWith(".urdf") && !utils::getBaseName(file_name).isEmpty());
  }
  else {
    save_button_->setEnabled(file_name.length() > 0);
  }
}
}  // namespace ui
}  // namespace urdf_builder
}  // namespace gui
