#pragma once

#include <QListWidget>

namespace qt
{
/**
 * ===== QListWidgetItem との違い =====
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

  /* アイテムを削除する． */
  void remove(QListWidgetItem* item);

  /* 指定したテキストのアイテムを選択する． */
  void setCurrentText(const QString& text);

  /* 何も選択しない状態にする． */
  void deselect();

  /* リストの高さを行数ぶんだけにする． */
  void shrinkToContents();

protected:
  void dropEvent(QDropEvent* event) override;
};

/**
 * ===== QListWidgetItem との違い =====
 * - UserRoleを基準に比較
 */
class ListWidgetItem : public QListWidgetItem
{
public:
  virtual bool operator<(const QListWidgetItem& rhs) const override;
};
}  // namespace qt
