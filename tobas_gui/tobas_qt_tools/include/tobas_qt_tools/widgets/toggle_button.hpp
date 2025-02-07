#pragma once

#include <QPushButton>

namespace qt
{
/* 押すたびにテキストが切り替わる単純なトグルボタン． */
class ToggleButton : public QPushButton
{
  Q_OBJECT

  using self = ToggleButton;
  using super = QPushButton;

Q_SIGNALS:
  void checked();
  void unchecked();

public:
  explicit ToggleButton(const QString& off_text, const QString& on_text, QWidget* parent = nullptr);

  void setChecked(bool _checked, bool block_signal = false);

private:
  const QString off_text_;
  const QString on_text_;

  bool checked_ = false;

private Q_SLOTS:
  void onClicked();
};
}  // namespace qt
