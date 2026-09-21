// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <QComboBox>
#include <QLabel>
#include <QPlainTextEdit>
#include <QProcess>
#include <QTextDecoder>
#include <QTimer>
#include <QWidget>

#include <tobas_qt_tools/widgets/toggle_button.hpp>

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
  ~FcConsoleWidget();

  void setEndpoint(const QString& host, const QString& user);
  bool isRunning() const;

  void start();
  void stop();
  void clear();

private:
  enum Status
  {
    kStopped,
    kWaiting,
    kReceiving,
    kStopping,
    kError,
  } status_ = kStopped;

  QString host_;
  QString user_;

  QString pending_output_;
  QProcess process_;
  QTimer flush_timer_;
  std::unique_ptr<QTextDecoder> decoder_;

  QComboBox* service_;
  qt::ToggleButton* start_stop_btn_;
  QLabel* status_label_;
  QPlainTextEdit* output_;

  void updateActions();
  void setStatus(Status status);

  void queueOutput(const QString& text);
  void readStandardOutput();
  void readStandardError();
  void readOutput();
  void flushOutput();
  void scrollToEnd();

  void onStartButtonClicked();
  void onStopButtonClicked();
  void onClearButtonClicked();
  void onSaveButtonClicked();

  void onWrapToggled(bool checked);
  void onProcessFinished(int code, QProcess::ExitStatus status);
  void onProcessErrorOccurred(QProcess::ProcessError error);

  friend QDebug operator<<(QDebug debug, const Status& status);
};
}  // namespace console
}  // namespace gui
}  // namespace tobas
