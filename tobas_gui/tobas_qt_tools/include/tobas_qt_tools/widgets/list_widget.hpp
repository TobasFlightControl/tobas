#pragma once

#include <QListWidget>

namespace qt
{
/**
 * ===== QListWidgetItemとの違い =====
 * - ドラッグアンドドロップでシグナル発行
 * - 追加メソッド
 */
class ListWidget : public QListWidget
{
  Q_OBJECT

  using super = QListWidget;

Q_SIGNALS:
  void itemMoved(QListWidgetItem* item);

public:
  using super::QListWidget;

  void remove(QListWidgetItem* item);

  /* リストにテキストが含まれる場合にtrueを返す． */
  bool contains(const QString& text);

  /* 選択中のアイテムのうち，最も上のものを返す．存在しない場合はNULLを返す． */
  QListWidgetItem* selectedItem();

protected:
  void dropEvent(QDropEvent* event) override;
};

/**
 * ===== QListWidgetItemとの違い =====
 * - UserRoleを基準にソート
 */
class ListWidgetItem : public QListWidgetItem
{
public:
  bool operator<(QListWidgetItem* rhs) const;
};
}  // namespace qt
