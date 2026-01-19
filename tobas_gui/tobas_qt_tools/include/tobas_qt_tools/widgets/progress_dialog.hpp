#pragma once

#include <QElapsedTimer>
#include <QProgressDialog>
#include <QTimer>

namespace qt
{
/**
 * ===== QProgressDialogとの違い =====
 * - ユーザーが他のUI要素と対話できないようにする
 * - タイトルを設定
 * - デフォルトで最小値に設定
 * - ダイアログに動作するインジケータを表示
 * - 追加メソッド
 */
class ProgressDialog : public QProgressDialog
{
  Q_OBJECT

  using self = ProgressDialog;
  using super = QProgressDialog;

public:
  explicit ProgressDialog(const QString& title = "", int num_steps = 1, QWidget* parent = nullptr);

  void show();
  void hide();

  void setLabelText(const QString& text);
  void setStep(int step);
  void progressStep();

private:
  const int num_steps_;

  int step_ = 0;
  QString text_;

  QTimer ui_timer_;
  QElapsedTimer wall_timer_;

private Q_SLOTS:
  void onTimerTimeout();
};
}  // namespace qt
