#pragma once

#include <QLabel>
#include <QVBoxLayout>

namespace tobas
{
namespace gui
{
namespace bm
{
class BaseConfigWidget : public QWidget
{
  Q_OBJECT

public:
  explicit BaseConfigWidget();

  /* タブに表示される名前． */
  virtual const char* name() const = 0;

  /* ページ上部に表示されるタイトル． */
  virtual const char* title() const = 0;

  /* 設定内容を初期化する． */
  virtual void reset() = 0;

protected:
  QVBoxLayout* rows_;

private:
  QLabel* title_;

private Q_SLOTS:
  void initialize();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
