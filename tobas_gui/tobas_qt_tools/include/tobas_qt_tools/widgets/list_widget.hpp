#pragma once

#include <QListWidget>
#include <QDropEvent>

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

  void dropEvent(QDropEvent* event) override;

  void remove(QListWidgetItem* item);

  /* 選択中のアイテムのうち，最も上のものを返す．存在しない場合はNULLを返す． */
  QListWidgetItem* selectedItem();
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
