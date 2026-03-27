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
      color = CYAN_PREFIX;
      label = "DEBUG";
      break;
    case QtInfoMsg:
      color = GREEN_PREFIX;
      label = "INFO";
      break;
    case QtWarningMsg:
      color = YELLOW_PREFIX;
      label = "WARN";
      break;
    case QtCriticalMsg:
      color = RED_PREFIX;
      label = "CRIT";
      break;
    case QtFatalMsg:
      color = MAGENTA_PREFIX;
      label = "FATAL";
      break;
    default:
      color = COLOR_RESET;
      label = "LOG";
      break;
  }

  QTextStream ts(stderr);
  ts << color << '[' << label << "] " << msg << COLOR_RESET;

  if (context.file && context.line > 0) {
    ts << " (" << context.file << ':' << context.line << ')';
  }
  ts << '\n';
  ts.flush();
}
}  // namespace qt
}  // namespace tobas
