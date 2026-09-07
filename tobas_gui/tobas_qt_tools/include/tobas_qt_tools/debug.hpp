// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTextStream>
#include <QtGlobal>

#include <tobas_std_tools/ansi_text_styles.hpp>

namespace tobas
{
namespace qt
{
void colorMessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
  QString color, label;
  switch (type) {
    case QtDebugMsg:
      color = st::kCyanPrefix;
      label = "DEBUG";
      break;
    case QtInfoMsg:
      color = st::kGreenPrefix;
      label = "INFO";
      break;
    case QtWarningMsg:
      color = st::kYellowPrefix;
      label = "WARN";
      break;
    case QtCriticalMsg:
      color = st::kRedPrefix;
      label = "CRIT";
      break;
    case QtFatalMsg:
      color = st::kMagentaPrefix;
      label = "FATAL";
      break;
    default:
      color = st::kColorReset;
      label = "LOG";
      break;
  }

  QTextStream ts(stderr);
  ts << color << '[' << label << "] " << msg << st::kColorReset;

  if (context.file && context.line > 0) {
    ts << " (" << context.file << ':' << context.line << ')';
  }
  ts << '\n';
  ts.flush();
}
}  // namespace qt
}  // namespace tobas
