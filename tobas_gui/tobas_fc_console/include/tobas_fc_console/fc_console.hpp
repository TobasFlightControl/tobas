// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <QComboBox>
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

  void setEndpoint(const QString& host, const QString& user);
  bool isRunning() const;

protected:
  void closeEvent(QCloseEvent* event) override;

private:
  QString host_;
  QString user_;

  bool running_ = false;
  QString pending_output_;
  QProcess process_;
  QTimer flush_timer_;
  std::unique_ptr<QTextDecoder> decoder_;

  QComboBox* service_;
  qt::ToggleButton* start_stop_btn_;
  QPlainTextEdit* output_;

  void updateActions();
  void setRunning(bool running);

  void queueOutput(const QString& text);
  void readStandardOutput();
  void readStandardError();
  void readOutput();
  void flushOutput();

  void start();
  void stop();
  void clear();
  void save();

  void onWrapToggled(bool checked);
  void onFinished(int code, QProcess::ExitStatus status);
  void onErrorOccurred(QProcess::ProcessError error);
};
}  // namespace console
}  // namespace gui
}  // namespace tobas
