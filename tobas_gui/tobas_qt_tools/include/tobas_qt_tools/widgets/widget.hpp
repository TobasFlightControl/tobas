#pragma once

#include <QWidget>

namespace tobas
{
namespace qt
{
/**
 * ===== QWidget との違い =====
 * - 追加メソッド
 */
class Widget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  using super::QWidget;

  QPoint getCenter() const;

  /* 枠内に収まるテキストの最大のポイントサイズを計算する． */
  int calcMaxTextPointSize(const QString& text, const QPoint& center) const;

  /* 背景色を設定する． */
  void setBackgroundColor(QPalette::ColorRole cr);

protected:
  /* テキストを書く． */
  void drawText(QPainter& painter, const QString& text, const QPoint& center, const QFont& font);

  /* テキストを書く． */
  void drawText(QPainter& painter, const QString& text, const QPoint& center, int point_size);

  /* 枠内に収まる最大サイズのテキストを書く． */
  void drawMaximumText(QPainter& painter, const QString& text, const QPoint& center);

  /* 枠内に収まる最大サイズのテキストを書く． */
  void drawMaximumText(QPainter& painter, const QString& text);
};
}  // namespace qt
}  // namespace tobas
