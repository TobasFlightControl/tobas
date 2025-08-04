#pragma once

#include <QLabel>
#include <QVBoxLayout>

namespace gui
{
namespace hw
{
class BaseHardwareSetupWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BaseHardwareSetupWidget();

  /* タブに表示される名前． */
  virtual const char* name() const = 0;

  /* ページ上部に表示されるタイトル． */
  virtual const char* title() const = 0;

  /* ロボットの構造を変えずにウィジェットを初期化する． */
  virtual void reset() = 0;

protected:
  QVBoxLayout* rows_;

private:
  QLabel* title_;

private Q_SLOTS:
  void initialize();
};
}  // namespace hw
}  // namespace gui
