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

  using self = FormLayout;
  using super = QFormLayout;

public:
  using QFormLayout::QFormLayout;

  /* ラベルを左中央に配置した行を追加する． */
  void addVAlignedRow(QWidget* label, QWidget* field);

  /* ラベルを左中央に配置した行を追加する． */
  void addVAlignedRow(const QString& label_text, QWidget* field);

  /* 全てのフォームを削除する． */
  void clear();

  /* 指定した行のラベルを取得する． */
  QWidget* getLabel(int row);

  /* 指定した行のウィジェットを取得する． */
  QWidget* getWidget(int row);
};
}  // namespace qt
