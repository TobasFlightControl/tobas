// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fc_console/fc_console.hpp"

#include <QCheckBox>
#include <QDateTime>
#include <QDebug>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QIODevice>
#include <QKeySequence>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollBar>
#include <QSet>
#include <QShortcut>
#include <QTextCursor>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>

namespace tobas::gui::console
{
FcConsoleWidget::FcConsoleWidget(QWidget* _parent) : QWidget(_parent)
{
  constexpr char kRealtimeService[] = "tobas_real_realtime.service";
  constexpr char kInterfaceService[] = "tobas_real_interface.service";

  service_ = new QComboBox();
  service_->addItem("Realtime", "-u " + QString(kRealtimeService));
  service_->addItem("Interface", "-u " + QString(kInterfaceService));
  service_->addItem("Both", "-u " + QString(kRealtimeService) + " -u " + QString(kInterfaceService));

  start_stop_btn_ = new qt::ToggleButton("Start", "Stop");
  status_label_ = new QLabel();

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
  controls->addWidget(start_stop_btn_);
  controls->addWidget(status_label_);
  controls->addStretch();
  controls->addWidget(wrap);
  controls->addWidget(clear_btn);
  controls->addWidget(save_btn);

  const auto rows = new QVBoxLayout();
  rows->addLayout(controls);
  rows->addWidget(output_);
  setLayout(rows);

  connect(start_stop_btn_, &qt::ToggleButton::checked, this, &self::onStartButtonClicked);
  connect(start_stop_btn_, &qt::ToggleButton::unchecked, this, &self::onStopButtonClicked);
  connect(clear_btn, &QPushButton::clicked, this, &self::onClearButtonClicked);
  connect(save_btn, &QPushButton::clicked, this, &self::onSaveButtonClicked);
  connect(wrap, &QCheckBox::toggled, this, &self::onWrapToggled);
  connect(&flush_timer_, &QTimer::timeout, this, &self::flushOutput);
  connect(&status_timer_, &QTimer::timeout, this, &self::onStatusAnimationTimeout);
  connect(&process_, &QProcess::readyReadStandardOutput, this, &self::readStandardOutput);
  connect(&process_, &QProcess::readyReadStandardError, this, &self::readStandardError);
  connect(&process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &self::onProcessFinished);
  connect(&process_, &QProcess::errorOccurred, this, &self::onProcessErrorOccurred);

  for (const auto key : { Qt::Key_Return, Qt::Key_Enter }) {
    const auto shortcut = new QShortcut(QKeySequence(key), this);
    shortcut->setContext(Qt::WidgetWithChildrenShortcut);
    connect(shortcut, &QShortcut::activated, this, &self::scrollToEnd);
  }

  flush_timer_.setInterval(50);
  status_timer_.setInterval(500);

  setStatus(kStopped);
}

FcConsoleWidget::~FcConsoleWidget()
{
  if (isRunning()) {
    process_.blockSignals(true);
    process_.terminate();
  }
}

void FcConsoleWidget::setEndpoint(const QString& _host, const QString& _user)
{
  if (host_ == _host && user_ == _user) {
    return;
  }

  if (isRunning()) {
    stop();
    qt::qWarnBox(this, "System log retrieval was stopped because the endpoint changed during retrieval.");
  }

  host_ = _host;
  user_ = _user;

  updateActions();
}

bool FcConsoleWidget::isRunning() const
{
  static const QSet running_statuses = { kWaiting, kReceiving, kStopping };
  return running_statuses.contains(status_);
}

void FcConsoleWidget::start()
{
  if (isRunning()) {
    qWarning() << "The journalctl process is already running.";
    return;
  }
  if (host_.isEmpty() || user_.isEmpty()) {
    qWarning() << "The endpoint has not been set yet.";
    return;
  }

  clear();
  stdout_decoder_.resetState();
  stderr_decoder_.resetState();

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

  setStatus(kWaiting);
  flush_timer_.start();
  process_.start("ssh", arguments, QIODevice::ReadOnly);
}

void FcConsoleWidget::stop()
{
  if (!isRunning()) {
    qWarning() << "The journalctl process is not running.";
    return;
  }

  setStatus(kStopping);
  process_.terminate();
}

void FcConsoleWidget::clear()
{
  pending_output_.clear();
  output_->clear();
}

void FcConsoleWidget::setStatus(Status _status)
{
  switch (_status) {
    case kStopped:
      status_text_.clear();
      break;
    case kWaiting:
      status_text_ = "Waiting for system logs";
      break;
    case kReceiving:
      status_text_ = "Receiving system logs";
      break;
    case kStopping:
      status_text_ = "Stopping";
      break;
    case kError:
      status_text_ = "Error";
      break;
    default:
      throw;
  }

  if (_status != kError) {
    status_label_->setToolTip({});
  }

  const auto old_running = isRunning();
  if (_status != status_) {
    qInfo() << "FC console status changed:" << status_ << "->" << _status;
    status_ = _status;
  }
  const auto new_running = isRunning();

  if (new_running) {
    if (!status_timer_.isActive()) {
      status_timer_.start();
    }
  }
  else {
    status_timer_.stop();
    status_animation_step_ = 0;
  }

  updateStatusLabel();
  updateActions();

  if (old_running != new_running) {
    Q_EMIT runningChanged(new_running);
  }
}

void FcConsoleWidget::updateStatusLabel()
{
  status_label_->setText(status_text_ + QString(status_animation_step_, '.'));
}

void FcConsoleWidget::updateActions()
{
  const auto running = isRunning();

  start_stop_btn_->setChecked(running);

  const auto endpoint_set = !host_.isEmpty() && !user_.isEmpty();
  switch (status_) {
    case kStopped:
      start_stop_btn_->setEnabled(endpoint_set);
      break;
    case kWaiting:
      start_stop_btn_->setEnabled(false);
      break;
    case kReceiving:
      start_stop_btn_->setEnabled(true);
      break;
    case kStopping:
      start_stop_btn_->setEnabled(false);
      break;
    case kError:
      start_stop_btn_->setEnabled(endpoint_set);
      break;
    default:
      throw;
  }

  service_->setEnabled(!running);
}

void FcConsoleWidget::queueOutput(const QString& _text)
{
  // Buffer SSH chunks so `flushOutput()` can batch display updates.
  // Updating the widget on every receive event would repeatedly lay out text and scroll,
  // making the UI less responsive when fetching a large journal history at startup.
  pending_output_ += _text;

  // Bound bursts as well as the document, without inserting annotations into the journal text.
  constexpr int kMaximumPendingCharacters = 2 * 1024 * 1024;
  if (pending_output_.size() > kMaximumPendingCharacters) {
    pending_output_ = pending_output_.right(kMaximumPendingCharacters);
  }
}

void FcConsoleWidget::readStandardOutput()
{
  const auto chunk = process_.readAllStandardOutput();
  if (!chunk.isEmpty() && status_ == kWaiting) {
    setStatus(kReceiving);
  }
  queueOutput(stdout_decoder_(chunk));
}

void FcConsoleWidget::readStandardError()
{
  queueOutput(stderr_decoder_(process_.readAllStandardError()));
}

void FcConsoleWidget::readOutput()
{
  readStandardOutput();
  readStandardError();
}

void FcConsoleWidget::flushOutput()
{
  if (pending_output_.isEmpty()) {
    return;
  }

  const auto scrollbar = output_->verticalScrollBar();
  const auto follow = scrollbar->value() == scrollbar->maximum();

  // Keep the user's selection and do not add newlines between arbitrary SSH chunks.
  QTextCursor cursor(output_->document());
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(pending_output_);
  pending_output_.clear();

  if (follow) {
    scrollToEnd();
  }
}

void FcConsoleWidget::scrollToEnd()
{
  const auto scrollbar = output_->verticalScrollBar();
  scrollbar->setValue(scrollbar->maximum());
}

void FcConsoleWidget::onStartButtonClicked()
{
  qDebug() << "FcConsoleWidget::onStartButtonClicked";

  start();
}

void FcConsoleWidget::onStopButtonClicked()
{
  qDebug() << "FcConsoleWidget::onStopButtonClicked";

  stop();
}

void FcConsoleWidget::onClearButtonClicked()
{
  qDebug() << "FcConsoleWidget::onClearButtonClicked";

  clear();
}

void FcConsoleWidget::onSaveButtonClicked()
{
  qDebug() << "FcConsoleWidget::onSaveButtonClicked";

  const auto timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
  const auto default_filename = timestamp + "_fc_console.log";
  const auto default_path = QDir(qt::expandUser(kGuiLogDir)).filePath(default_filename);
  const auto path = QFileDialog::getSaveFileName(this, "Save FC Console", default_path, "Log files (*.log *.txt)");
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

void FcConsoleWidget::onWrapToggled(bool _checked)
{
  qDebug().nospace() << "FcConsoleWidget::onWrapToggled(" << _checked << ")";

  output_->setLineWrapMode(_checked ? QPlainTextEdit::WidgetWidth : QPlainTextEdit::NoWrap);
}

void FcConsoleWidget::onStatusAnimationTimeout()
{
  ++status_animation_step_ %= 4;
  updateStatusLabel();
}

void FcConsoleWidget::onProcessFinished(int _code, QProcess::ExitStatus _status)
{
  qDebug().nospace() << "FcConsoleWidget::onProcessFinished(" << _code << ", " << _status << ")";

  readOutput();
  flushOutput();

  flush_timer_.stop();
  const auto failed = status_ != kStopping && (status_ == kError || _status == QProcess::CrashExit || _code != 0);
  setStatus(failed ? kError : kStopped);
}

void FcConsoleWidget::onProcessErrorOccurred(QProcess::ProcessError _error)
{
  qDebug().nospace() << "FcConsoleWidget::onProcessErrorOccurred(" << _error << ")";

  if (status_ == kStopping && _error != QProcess::FailedToStart) {
    return;
  }

  setStatus(kError);
  status_label_->setToolTip(process_.errorString());

  if (_error == QProcess::FailedToStart) {
    flush_timer_.stop();
  }
}

QDebug operator<<(QDebug _debug, FcConsoleWidget::Status _status)
{
  const QDebugStateSaver saver(_debug);

  switch (_status) {
    case FcConsoleWidget::kStopped:
      _debug << "Stopped";
      break;
    case FcConsoleWidget::kWaiting:
      _debug << "Waiting";
      break;
    case FcConsoleWidget::kReceiving:
      _debug << "Receiving";
      break;
    case FcConsoleWidget::kStopping:
      _debug << "Stopping";
      break;
    case FcConsoleWidget::kError:
      _debug << "Error";
      break;
    default:
      throw;
  }

  return _debug;
}
}  // namespace tobas::gui::console
