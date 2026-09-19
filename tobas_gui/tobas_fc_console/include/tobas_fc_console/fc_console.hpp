// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <QComboBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QTextDecoder>
#include <QTimer>
#include <QWidget>

namespace tobas
{
namespace gui
{
namespace console
{
class FcConsoleWidget : public QWidget
{
  Q_OBJECT

  using self = FcConsoleWidget;

Q_SIGNALS:
  void runningChanged(bool running);

public:
  explicit FcConsoleWidget(QWidget* parent = nullptr);
  ~FcConsoleWidget() override;

  void setEndpoint(const QString& host, const QString& user);
  bool isRunning() const;

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  QString host_;
  QString user_;
  bool running_ = false;
  bool stopping_ = false;

  QProcess process_;
  QTimer flush_timer_;
  QTimer kill_timer_;
  std::unique_ptr<QTextDecoder> stdout_decoder_;
  std::unique_ptr<QTextDecoder> stderr_decoder_;
  QString pending_output_;

  QComboBox* service_;
  QPushButton* start_btn_;
  QPushButton* stop_btn_;
  QPlainTextEdit* output_;

  void updateActions();
  void setRunning(bool running);

  void queueOutput(const QString& text);
  void readOutput();
  void flushOutput();

  void start();
  void stop();
  void save();

  void onClearButtonClicked();
  void onWrapToggled(bool checked);
  void onKillTimeout();
  void onStarted();
  void onFinished();
  void onErrorOccurred(QProcess::ProcessError error);
};
}  // namespace console
}  // namespace gui
}  // namespace tobas
