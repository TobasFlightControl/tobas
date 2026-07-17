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
 * ===== Differences from `QProgressDialog` =====
 * - Prevent the user from interacting with other UI elements
 * - Hide the title bar buttons
 * - Set title
 * - Set to the minimum value by default
 * - Show a spinner to the right of the label text
 * - Additional methods
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

  int spinner_step_ = 0;
  QTimer timer_;

private Q_SLOTS:
  void onTimerTimeout();
};
}  // namespace qt
}  // namespace tobas
