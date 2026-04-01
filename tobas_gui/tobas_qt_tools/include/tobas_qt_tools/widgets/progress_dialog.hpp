// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QProgressDialog>
#include <QTimer>

namespace tobas
{
namespace qt
{
/**
 * ===== QProgressDialog との違い =====
 * - ユーザーが他のUI要素と対話できないようにする
 * - タイトルを設定
 * - デフォルトで最小値に設定
 * - ラベルテキストの右にスピナーを表示
 * - 追加メソッド
 */
class ProgressDialog : public QProgressDialog
{
  Q_OBJECT

  using self = ProgressDialog;
  using super = QProgressDialog;

  static constexpr int kSpinnerFrameSize = 4;
  static constexpr char kSpinnerFrames[] = "|/-\\";

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

  int spinner_step_ = 0;
  QTimer timer_;

private Q_SLOTS:
  void onTimerTimeout();
};
}  // namespace qt
}  // namespace tobas
