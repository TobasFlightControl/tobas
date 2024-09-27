#pragma once

#include <QWidget>

namespace qt
{
/**
 * ===== QWidgetとの違い =====
 * - 追加メソッド
 */
class Widget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  using super::QWidget;

  QPoint getCenter() const;

protected:
  /* 枠内に収まる最大サイズのテキストを書く． */
  void drawMaximumText(QPainter& painter, const QString& text);

  /* 枠内に収まる最大サイズのテキストを書く． */
  void drawMaximumText(QPainter& painter, const QString& text, const QPoint& center);
};
}  // namespace qt
