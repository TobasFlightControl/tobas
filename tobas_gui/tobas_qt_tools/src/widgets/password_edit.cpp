#include "tobas_qt_tools/widgets/password_edit.hpp"

#include <QAction>
#include <QDebug>
#include <QFile>

#include "tobas_qt_tools/path.hpp"

namespace qt
{
PasswordEdit::PasswordEdit(QWidget* parent) : super(parent)
{
  const auto rsrc_path = getResourcePath();
  eye_on_ = QIcon(rsrc_path + "/eye_on.png");
  eye_off_ = QIcon(rsrc_path + "/eye_off.png");

  toggle_ = addAction(eye_off_, QLineEdit::TrailingPosition);
  toggle_->setCheckable(true);

  setMode(false);

  connect(toggle_, &QAction::toggled, this, &self::onIconToggled);
}

void PasswordEdit::setMode(bool on)
{
  setEchoMode(on ? QLineEdit::Normal : QLineEdit::Password);
  toggle_->setIcon(on ? eye_on_ : eye_off_);
  toggle_->setToolTip(on ? "Hide password" : "Show password");
}

void PasswordEdit::onIconToggled(bool on)
{
  const auto cursor_pos = cursorPosition();  // カーソルの位置を保持
  setMode(on);
  setCursorPosition(cursor_pos);
}
}  // namespace qt
