#pragma once

#include <QFormLayout>

namespace qt
{
/**
 * ===== QFormLayoutとの違い =====
 * - 追加メソッド
 */
class FormLayout : public QFormLayout
{
  Q_OBJECT

public:
  /* 全てのフォームを削除する． */
  void clear();

  /* 指定した行のラベルを取得する． */
  QWidget* getLabel(int row);

  /* 指定した行のウィジェットを取得する． */
  QWidget* getWidget(int row);
};
}  // namespace qt
