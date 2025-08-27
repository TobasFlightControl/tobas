#pragma once

#include <QDoubleSpinBox>

namespace qt
{
/**
 * ===== QDoubleSpinBoxとの違い =====
 * - 最大最小のデフォルト値をdoubleの最大最小に設定
 * - マウスホイールイベントを無効化
 * - フォーカス時にテキスト全体を選択
 */
class DoubleSpinBox : public QDoubleSpinBox
{
  Q_OBJECT

  using super = QDoubleSpinBox;

public:
  explicit DoubleSpinBox(QWidget* parent = nullptr);

protected:
  void wheelEvent(QWheelEvent* event) override;
  void focusInEvent(QFocusEvent* event) override;
};
}  // namespace qt
