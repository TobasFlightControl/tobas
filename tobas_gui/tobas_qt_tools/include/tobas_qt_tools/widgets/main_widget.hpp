#pragma once

#include <QWidget>
#include <QCloseEvent>

namespace qt
{
class MainWidget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  explicit MainWidget(const QString& title, const QString& icon_path, QWidget* widget);

  void closeEvent(QCloseEvent* event) override;

private:
  QWidget* widget_;
};
}  // namespace qt
