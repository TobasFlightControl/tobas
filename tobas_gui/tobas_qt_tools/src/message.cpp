#include <QMessageBox>

#include "tobas_qt_tools/message.hpp"

namespace qt
{
void qInfoBox(QWidget* parent, const QString& msg)
{
  QMessageBox::information(parent, "INFO", msg);
}

void qWarnBox(QWidget* parent, const QString& msg)
{
  QMessageBox::warning(parent, "WARN", msg);
}

void qErrorBox(QWidget* parent, const QString& msg)
{
  QMessageBox::critical(parent, "ERROR", msg);
}

bool yesOrNo(QWidget* parent, const QString& text, QMessageLevel level)
{
  QMessageBox msg_box(parent);

  // メッセージレベルを設定
  switch (level)
  {
    case QMessageLevel::INFO:
      msg_box.setIcon(QMessageBox::Icon::Information);
      msg_box.setWindowTitle("INFO");
      break;
    case QMessageLevel::WARN:
      msg_box.setIcon(QMessageBox::Icon::Warning);
      msg_box.setWindowTitle("WARN");
      break;
    case QMessageLevel::ERROR:
      msg_box.setIcon(QMessageBox::Icon::Critical);
      msg_box.setWindowTitle("ERROR");
      break;
    default:
      throw;
  }

  // テキストの設定
  msg_box.setText(text);

  // テキストの設定
  msg_box.setText(text);

  // ボタンの設定
  // 配置は自動で決まる．明確な規則は無いが，全体でルールを統一することが大事: https://nanika.design/blog/1162/
  msg_box.setStandardButtons(QMessageBox::StandardButton::Yes | QMessageBox::StandardButton::No);
  msg_box.setDefaultButton(QMessageBox::StandardButton::No);

  // ユーザの返事を取得し，Yesの場合にTrueを返す
  return msg_box.exec() == QMessageBox::StandardButton::Yes;
}
}  // namespace qt
