#include <csignal>
#include <QVBoxLayout>
#include <QIcon>
#include <QCloseEvent>

#include "tobas_qt_tools/widgets/main_widget.hpp"

namespace qt
{
MainWidget::MainWidget(const QString& title, const QString& icon_path, QWidget* widget) : widget_(widget)
{
  setWindowTitle(title);
  setWindowIcon(QIcon(icon_path));

  auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(widget);
}

void MainWidget::closeEvent(QCloseEvent* event)
{
  widget_->close();
  event->accept();

  // クローズ時にプロセスごと落とすことで確実に終了させる
  kill(getpid(), SIGINT);
}
}  // namespace qt
