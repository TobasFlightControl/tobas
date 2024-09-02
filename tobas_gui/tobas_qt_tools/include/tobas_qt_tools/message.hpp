#pragma once

#include <QWidget>

namespace qt
{
enum QMessageLevel
{
  INFO,
  WARN,
  ERROR,
};

void qInfoBox(QWidget* parent, const QString& msg);
void qWarnBox(QWidget* parent, const QString& msg);
void qErrorBox(QWidget* parent, const QString& msg);

/* Yes/No型の質問を含むダイアログを表示し，Yesの場合にtrueを返す． */
bool yesOrNo(QWidget* parent, const QString& text, QMessageLevel level);
}  // namespace qt
