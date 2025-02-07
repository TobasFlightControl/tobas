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

void ToggleButton::setChecked(bool _checked, bool block_signal)
{
  // 状態が変わらないなら何もしない
  if (_checked == checked_)
    return;

  // 状態を更新
  checked_ = _checked;

  // 新しい状態に応じたテキスト設定とシグナル発行
  // XXX: スロットの呼び出しはキューではなく割り込みになることがあるため，必ず関数の最後に発行する．
  if (_checked)
  {
    setText(on_text_);
    if (!block_signal)
      Q_EMIT checked();
  }
  else
  {
    setText(off_text_);
    if (!block_signal)
      Q_EMIT unchecked();
  }
}

void ToggleButton::onClicked()
{
  const auto next_check_state = !checked_;
  setChecked(next_check_state);
}
}  // namespace qt
