#include <QDebug>

#include "tobas_qt_tools/widgets/toggle_button.hpp"

namespace qt
{
ToggleButton::ToggleButton(const QString& off_text, const QString& on_text, QWidget* parent)
  : super(parent), off_text_(off_text), on_text_(on_text)
{
  setText(off_text);

  connect(this, &super::clicked, this, &self::onClicked);
}

void ToggleButton::setChecked(bool _checked)
{
  // 状態が変わらないなら何もしない
  if (_checked == checked_) {
    return;
  }

  // 状態を更新
  checked_ = _checked;

  // 新しい状態に応じたテキスト設定
  // シグナル発行は行わない
  if (_checked) {
    setText(on_text_);
  }
  else {
    setText(off_text_);
  }
}

void ToggleButton::onClicked()
{
  setChecked(!checked_);

  if (checked_) {
    Q_EMIT checked();
  }
  else {
    Q_EMIT unchecked();
  }
}
}  // namespace qt
