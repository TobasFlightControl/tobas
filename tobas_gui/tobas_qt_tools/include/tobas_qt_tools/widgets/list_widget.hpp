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

  /* リストにテキストが含まれる場合にtrueを返す． */
  bool contains(const QString& text) const;

  /* 選択中のアイテムのうち，最も上のものを返す．存在しない場合はNULLを返す． */
  QListWidgetItem* selectedItem();
  const QListWidgetItem* selectedItem() const;

  /* アイテムを削除する． */
  void remove(QListWidgetItem* item);

  /* 何も選択しない状態にする． */
  void deselect();

  /* リストの高さを行数ぶんだけにする． */
  void shrinkToContents();

protected:
  void dropEvent(QDropEvent* event) override;
};

/**
 * ===== QListWidgetItemとの違い =====
 * - UserRoleを基準に比較
 */
class ListWidgetItem : public QListWidgetItem
{
public:
  virtual bool operator<(const QListWidgetItem& rhs) const override;
};
}  // namespace qt
