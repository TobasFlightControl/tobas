// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fc_console/fc_console.hpp"

#include <QCheckBox>
#include <QDebug>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QIODevice>
#include <QSaveFile>
#include <QScrollBar>
#include <QTextCursor>
#include <QVBoxLayout>

#include <tobas_qt_tools/message.hpp>

namespace tobas
{
namespace gui
{
namespace console
{
FcConsoleWidget::FcConsoleWidget(QWidget* parent) : QWidget(parent)
{
  constexpr char kRealtimeService[] = "tobas_real_realtime.service";
  constexpr char kInterfaceService[] = "tobas_real_interface.service";

  service_ = new QComboBox();
  service_->addItem("Realtime", "-u " + QString(kRealtimeService));
  service_->addItem("Interface", "-u " + QString(kInterfaceService));
  service_->addItem("Both", "-u " + QString(kRealtimeService) + " -u " + QString(kInterfaceService));

  start_btn_ = new QPushButton("Start");
  stop_btn_ = new QPushButton("Stop");

  const auto wrap = new QCheckBox("Wrap lines");
  const auto clear_btn = new QPushButton("Clear");
  const auto save_btn = new QPushButton("Save");

  output_ = new QPlainTextEdit();
  output_->setReadOnly(true);
  output_->setUndoRedoEnabled(false);
  output_->setLineWrapMode(QPlainTextEdit::NoWrap);
  output_->setMaximumBlockCount(50000);

  const auto controls = new QHBoxLayout();
  controls->addWidget(service_);
  controls->addWidget(start_btn_);
  controls->addWidget(stop_btn_);
  controls->addStretch();
  controls->addWidget(wrap);
  controls->addWidget(clear_btn);
  controls->addWidget(save_btn);

  const auto rows = new QVBoxLayout();
  rows->addLayout(controls);
  rows->addWidget(output_);
  setLayout(rows);

  flush_timer_.setInterval(50);

  connect(start_btn_, &QPushButton::clicked, this, &self::start);
  connect(stop_btn_, &QPushButton::clicked, this, &self::stop);
  connect(save_btn, &QPushButton::clicked, this, &self::save);
  connect(clear_btn, &QPushButton::clicked, this, &self::onClearButtonClicked);
  connect(wrap, &QCheckBox::toggled, this, &self::onWrapToggled);
  connect(&flush_timer_, &QTimer::timeout, this, &self::flushOutput);
  connect(&process_, &QProcess::readyReadStandardOutput, this, &self::readOutput);
  connect(&process_, &QProcess::readyReadStandardError, this, &self::readOutput);
  connect(&process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &self::onFinished);
  connect(&process_, &QProcess::errorOccurred, this, &self::onErrorOccurred);

  updateActions();
}

void FcConsoleWidget::setEndpoint(const QString& host, const QString& user)
{
  if (host_ == host && user_ == user) {
    return;
  }

  stop();

  host_ = host;
  user_ = user;

  updateActions();
}

bool FcConsoleWidget::isRunning() const
{
  return running_;
}

void FcConsoleWidget::closeEvent(QCloseEvent* event)
{
  stop();
  event->accept();
}

void FcConsoleWidget::updateActions()
{
  start_btn_->setEnabled(!running_ && !host_.isEmpty() && !user_.isEmpty());
  stop_btn_->setEnabled(running_);
  service_->setEnabled(!running_);
}

void FcConsoleWidget::setRunning(bool running)
{
  running_ = running;
  updateActions();
  Q_EMIT runningChanged(running_);
}

void FcConsoleWidget::queueOutput(const QString& text)
{
  pending_output_ += text;

  // Bound bursts as well as the document, without inserting annotations into the journal text.
  constexpr int kMaximumPendingCharacters = 2 * 1024 * 1024;
  if (pending_output_.size() > kMaximumPendingCharacters) {
    pending_output_ = pending_output_.right(kMaximumPendingCharacters);
  }
}

void FcConsoleWidget::readOutput()
{
  queueOutput(stdout_decoder_->toUnicode(process_.readAllStandardOutput()));
  queueOutput(stderr_decoder_->toUnicode(process_.readAllStandardError()));
}

void FcConsoleWidget::flushOutput()
{
  if (pending_output_.isEmpty()) {
    return;
  }

  // Keep the user's selection and do not add newlines between arbitrary SSH chunks.
  QTextCursor cursor(output_->document());
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(pending_output_);
  pending_output_.clear();

  const auto scrollbar = output_->verticalScrollBar();
  scrollbar->setValue(scrollbar->maximum());
}

void FcConsoleWidget::start()
{
  if (running_ || host_.isEmpty() || user_.isEmpty()) {
    return;
  }

  // A single journalctl invocation reads history and follows it without a handover gap.
  const auto command = "journalctl -b --no-pager -o short-monotonic -n all -f " + service_->currentData().toString();
  const QStringList arguments = {
    "-T",    "-n",
    "-o",    "BatchMode=yes",
    "-o",    "StrictHostKeyChecking=accept-new",
    "-o",    "ConnectTimeout=10",
    "-o",    "ServerAliveInterval=5",
    "-o",    "ServerAliveCountMax=3",
    "-l",    user_,
    "--",    host_,
    command,
  };

  pending_output_.clear();
  output_->clear();
  stdout_decoder_.reset(QTextCodec::codecForName("UTF-8")->makeDecoder());
  stderr_decoder_.reset(QTextCodec::codecForName("UTF-8")->makeDecoder());
  setRunning(true);
  flush_timer_.start();
  process_.start("ssh", arguments, QIODevice::ReadOnly);
}

void FcConsoleWidget::stop()
{
  if (!running_) {
    return;
  }

  updateActions();
  process_.terminate();
}

void FcConsoleWidget::save()
{
  const auto path = QFileDialog::getSaveFileName(this, "Save FC console", "fc-console.log", "Log files (*.log *.txt)");
  if (path.isEmpty()) {
    return;
  }
  flushOutput();
  QSaveFile file(path);
  const auto contents = output_->toPlainText().toUtf8();
  if (!file.open(QIODevice::WriteOnly) || file.write(contents) != contents.size() || !file.commit()) {
    qt::qErrorBox(this, "Failed to save output: " + file.errorString());
  }
}

void FcConsoleWidget::onClearButtonClicked()
{
  pending_output_.clear();
  output_->clear();
}

void FcConsoleWidget::onWrapToggled(bool checked)
{
  output_->setLineWrapMode(checked ? QPlainTextEdit::WidgetWidth : QPlainTextEdit::NoWrap);
}

void FcConsoleWidget::onFinished(int, QProcess::ExitStatus)
{
  readOutput();
  flushOutput();
  flush_timer_.stop();
  setRunning(false);
}

void FcConsoleWidget::onErrorOccurred(QProcess::ProcessError error)
{
  if (error == QProcess::FailedToStart) {
    flush_timer_.stop();
    setRunning(false);
  }
}
}  // namespace console
}  // namespace gui
}  // namespace tobas
