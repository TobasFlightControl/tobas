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
void colorMessageHandler(QtMsgType _type, const QMessageLogContext& _context, const QString& _msg)
{
  QString color, label;
  switch (_type) {
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
  ts << color << '[' << label << "] " << _msg << st::kColorReset;

  if (_context.file && _context.line > 0) {
    ts << " (" << _context.file << ':' << _context.line << ')';
  }
  ts << '\n';
  ts.flush();
}
}  // namespace qt
}  // namespace tobas
