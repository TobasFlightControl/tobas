#pragma once

#include <QProgressDialog>

namespace qt
{
/**
 * ===== QProgressDialogとの違い =====
 * - ユーザーが他のUI要素と対話できないようにする
 * - タイトルを設定
 * - デフォルトで最小値に設定
 * - 各操作後に画面更新のためスリープ
 * - 追加メソッド
 */
class ProgressDialog : public QProgressDialog
{
  Q_OBJECT

  using super = QProgressDialog;

  static constexpr size_t kRefleshSleep = 50;  // [ms]

public:
  explicit ProgressDialog(const QString& title = "", int num_steps = 1, QWidget* parent = nullptr);

  void show();
  void setValue(int value);
  void setLabelText(const QString& text);
  void setStep(int step);
  void progressStep();
  void reflesh();

private:
  const int num_steps_;
  int step_ = 0;
};
}  // namespace qt
