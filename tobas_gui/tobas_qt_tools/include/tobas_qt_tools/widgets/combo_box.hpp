#pragma once

#include <QComboBox>

namespace qt
{
/**
 * ===== QComboBoxとの違い =====
 * - マウスホイールイベントを無効化
 * - setCurrentIndexで範囲チェック
 * - setCurrentTextで存在しない選択肢を指定するとエラー
 * - 追加メソッド
 */
class ComboBox : public QComboBox
{
  Q_OBJECT

  using super = QComboBox;

public:
  using super::QComboBox;

  void wheelEvent(QWheelEvent* event) override;

  bool contains(const QString& text) const;

  void removeText(const QString& text);

  void setItemEnabled(int row, bool enabled);
  void setItemEnabled(const QString& text, bool enabled);

public Q_SLOTS:
  void setCurrentIndex(int index);
  void setCurrentText(const QString& text);
};
}  // namespace qt
